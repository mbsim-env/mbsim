import subprocess
import random
import math
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from collections import deque
import fmpy
import os
from scipy.integrate import RK45

load = False
save = True
training = True
simulation = True

nz = 4
nO = 4
nA = 1
sA = 10
tau = 0.005
gamma = 0.99
batchSize = 128
maxLen = 10000
nEpisodes = 0 if (not training and simulation) else 500
tEnd = 10
atol = 1e-6
rtol = 1e-3
epsStart = 0.9
epsEnd = 0.05
epsDecay = 2000
fmuPlot = False

class Value(nn.Module):

  def __init__(self, nO, nA):
    super(Value, self).__init__()
    self.layer1 = nn.Linear(nO+nA, 128)
    self.layer2 = nn.Linear(128, 128)
    self.layer3 = nn.Linear(128, 1)

  def forward(self, x):
    x = F.relu(self.layer1(x))
    x = F.relu(self.layer2(x))
    return self.layer3(x)

class Actor(nn.Module):

  def __init__(self, nO, nA, scale):
    super(Actor, self).__init__()
    self.layer1 = nn.Linear(nO, 128)
    self.layer2 = nn.Linear(128, 128)
    self.layer3 = nn.Linear(128, nA)
    self.scale = scale

  def forward(self, x):
    x = F.relu(self.layer1(x))
    x = F.relu(self.layer2(x))
    return self.scale*F.tanh(self.layer3(x))

def setTime(t):
  global fmu
  fmu.setTime(t)

def setState(z):
  global fmu, nz
  _pz = z.ctypes.data_as(fmpy.fmi1.POINTER(fmpy.fmi1.c_double))
  fmu.setContinuousStates(_pz, nz)

def getValue(index):
  global fmu
  return np.array(fmu.getReal(index))

def setValue(index,value):
  global fmu
  fmu.setReal(vr=index, value=value)

def dgl(t,z):
  global fmu, nz
  zd = np.zeros(nz)
  _pz = z.ctypes.data_as(fmpy.fmi1.POINTER(fmpy.fmi1.c_double))
  _pzd = zd.ctypes.data_as(fmpy.fmi1.POINTER(fmpy.fmi1.c_double))
  fmu.setTime(t)
  fmu.setContinuousStates(_pz, nz)
  fmu.getDerivatives(_pzd, nz)
  return zd

valueNet = Value(nO, nA)
actorNet = Actor(nO, nA, sA)
valueTargetNet = Value(nO, nA)
actorTargetNet = Actor(nO, nA, sA)
if load:
  valueNet.load_state_dict(torch.load("value_net.pt", weights_only=True))
  actorNet.load_state_dict(torch.load("actor_net.pt", weights_only=True))
valueTargetNet.load_state_dict(valueNet.state_dict())
actorTargetNet.load_state_dict(actorNet.state_dict())

memory = deque([], maxlen=maxLen)
valueOptimizer = optim.AdamW(valueNet.parameters(), lr=1e-4, amsgrad=True)
actorOptimizer = optim.AdamW(actorNet.parameters(), lr=1e-4, amsgrad=True)

def evalAction(state):
  epsThreshold = epsEnd+(epsStart-epsEnd)*math.exp(-steps/epsDecay)
  sample = random.random()
  if not training or sample>epsThreshold:
    with torch.no_grad():
      return actorNet(torch.tensor(state, dtype=torch.float32)).numpy()
  else:
    return np.random.uniform(-sA,sA,nA)

def optimize_model():
  if len(memory) < batchSize:
    return
  batch = np.array(random.sample(memory,batchSize))
  stateBatch = batch[:,0:nO]
  stateActionBatch = batch[:,0:nO+nA]
  nextStateBatch = batch[:,nO+nA:2*nO+nA]
  rewardBatch = batch[:,2*nO+nA]

  with torch.no_grad():
    nextActionBatch = actorTargetNet(torch.tensor(nextStateBatch,dtype=torch.float32))
  nextStateActionBatch = torch.cat((torch.tensor(nextStateBatch,dtype=torch.float32),nextActionBatch),1)

  stateActionValues = valueNet(torch.tensor(stateActionBatch,dtype=torch.float32)).squeeze()

  with torch.no_grad():
    nextStateValues = valueTargetNet(nextStateActionBatch).squeeze()
  expectedStateActionValues = (nextStateValues * gamma) + torch.tensor(rewardBatch,dtype=torch.float32)

  valueLoss = ((stateActionValues - expectedStateActionValues)**2).mean()
  valueOptimizer.zero_grad()
  valueLoss.backward()
  torch.nn.utils.clip_grad_value_(valueNet.parameters(), 100)
  valueOptimizer.step()

  currentActionBatch = actorNet(torch.tensor(stateBatch, dtype=torch.float32))
  currentStateActionBatch = torch.cat((torch.tensor(stateBatch, dtype=torch.float32),currentActionBatch),1)
  currentStateActionValues = valueNet(currentStateActionBatch).squeeze()

  actorLoss = -currentStateActionValues.mean()
  actorOptimizer.zero_grad()
  actorLoss.backward()
  torch.nn.utils.clip_grad_value_(actorNet.parameters(), 100)
  actorOptimizer.step()

steps = 0

fmuFileName = 'mbsim.fmu'
if not os.path.isfile(fmuFileName):
  print("create fmu")
  subprocess.run(["mbsimCreateFMU", "--modulePath", "mbsimReinforcementLearning", "--noparam", "MBS.mbsx"])
fmuPath = os.path.abspath('.')+'/fmu'
if os.path.exists(fmuPath):
  print("use existing fmu")
  unzipdir = fmuPath
else:
  print("extract fmu")
  unzipdir = fmpy.extract(fmuFileName,fmuPath)

model_description = fmpy.read_model_description(fmuFileName)
fmu_args = {'guid': model_description.guid, 'modelIdentifier': model_description.modelExchange.modelIdentifier, 'unzipDirectory': unzipdir}

fmu = fmpy.fmi1.FMU1Model(**fmu_args)

fmu.instantiate()

vrs = {}
for variable in model_description.modelVariables:vrs[variable.name] = variable.valueReference

#iP = [vrs["training"]]
iO = [vrs["Links.'Observation'[0]"], vrs["Links.'Observation'[1]"], vrs["Links.'Observation'[2]"], vrs["Links.'Observation'[3]"]]
iT = [vrs["Links.'Termination'"]]
iR = [vrs["Links.'Reward'"]]
iA = [vrs["Links.'Action'"]]

#fmu.setReal(vr=iP, value=[training])
fmu.initialize()

dz = np.zeros(nz)
_pdz = dz.ctypes.data_as(fmpy.fmi1.POINTER(fmpy.fmi1.c_double))

nMaxSteps = 0

for n in range(0,nEpisodes):
  z0 = np.random.uniform(-0.05,0.05,nz)
  rk45 = RK45(dgl, 0, z0, tEnd, atol=atol, rtol=rtol)
  i = 0
  observation = getValue(iO)
  action = evalAction(observation)

  while rk45.t < tEnd-1e-2:
    setValue(iA,action)

    rk45.step()

    if fmuPlot:
      fmu.completedIntegratorStep()

    termination = False

    setTime(rk45.t)
    setState(rk45.y)
    nextObservation = getValue(iO)
    reward = getValue(iR)
    termination = bool(getValue(iT)[0])
    i += 1

    if training:
      memory.append(np.block([observation, action, nextObservation, reward]))
      observation = nextObservation
      action = evalAction(observation)

      optimize_model()

      valueTargetNetStateDict = valueTargetNet.state_dict()
      valueNetStateDict = valueNet.state_dict()
      for key in valueNetStateDict:
        valueTargetNetStateDict[key] = valueNetStateDict[key]*tau + valueTargetNetStateDict[key]*(1-tau)
      valueTargetNet.load_state_dict(valueTargetNetStateDict)

      actorTargetNetStateDict = actorTargetNet.state_dict()
      actorNetStateDict = actorNet.state_dict()
      for key in actorNetStateDict:
        actorTargetNetStateDict[key] = actorNetStateDict[key]*tau + actorTargetNetStateDict[key]*(1-tau)
      actorTargetNet.load_state_dict(actorTargetNetStateDict)

    if termination:
      break

  steps += i
  print(f"episode {n}, time {rk45.t:.1f}, steps {i}, total steps {steps}")

  if rk45.t>=tEnd-1e-2:
    nMaxSteps += 1
  else:
    nMaxSteps = 0
  if nMaxSteps >= 5:
    break

fmu.terminate()

if save:
  torch.save(valueNet.state_dict(), "value_net.pt")
  torch.save(actorNet.state_dict(), "actor_net.pt")

if simulation:
  if os.path.isfile("actor_net.pt"):
    subprocess.run(["mbsimxml", "--modulePath", "mbsimReinforcementLearning", "training=False", "MBS.mbsx"])
    if os.path.isfile("MBS.ombvx"):
      subprocess.run(["openmbv", "MBS.ombvx"])
  else:
    print("Simulation can not be started. File \"actor_net.pt\" does not exist.")

import random
import math
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
import torch.nn.functional as F
from collections import deque
import fmpy

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

nO = 4
nA = 1
sA = 10
nS = 3
batchSize = 128
tau = 0.005
gamma = 0.99
valueNet = Value(nO, nA)
actorNet = Actor(nO, nA, sA)
valueTargetNet = Value(nO, nA)
actorTargetNet = Actor(nO, nA, sA)
valueTargetNet.load_state_dict(valueNet.state_dict())
actorTargetNet.load_state_dict(actorNet.state_dict())
memory = deque([], maxlen=10000)
valueOptimizer = optim.AdamW(valueNet.parameters(), lr=1e-4, amsgrad=True)
actorOptimizer = optim.AdamW(actorNet.parameters(), lr=1e-4, amsgrad=True)
def evalAction(state):
  with torch.no_grad():
    a = actorNet(torch.tensor(state, dtype=torch.float32)).numpy()
    a += nS*np.random.randn(nA)
  return np.clip(a, -sA, sA)
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

training = True
steps = 0

fmu_filename = 'mbsim.fmu'
model_description = fmpy.read_model_description(fmu_filename)
unzipdir = fmpy.extract(fmu_filename)
fmu_args = {'guid': model_description.guid, 'modelIdentifier': model_description.modelExchange.modelIdentifier, 'unzipDirectory': unzipdir}
fmu = fmpy.fmi1.FMU1Model(**fmu_args)
fmu.instantiate()
vrs = {}
for variable in model_description.modelVariables:vrs[variable.name] = variable.valueReference
iO = [vrs["Links.'Observation'[0]"], vrs["Links.'Observation'[1]"], vrs["Links.'Observation'[2]"], vrs["Links.'Observation'[3]"]]
iA = [vrs["Links.'Action'"]]
fmu.initialize()

nz = 4
dz = np.zeros(nz)
_pdz = dz.ctypes.data_as(fmpy.fmi1.POINTER(fmpy.fmi1.c_double))

dt = 0.02

for n in range(0,500):
  t = 0
  fmu.setTime(t)
  z = np.random.uniform(-0.05,0.05,nz)
  _pz = z.ctypes.data_as(fmpy.fmi1.POINTER(fmpy.fmi1.c_double))
  fmu.setContinuousStates(_pz,nz)

  for i in range(0,int(10/dt)):
    observation = np.array(fmu.getReal(iO))
    action = evalAction(observation)
    fmu.setReal(vr=iA, value=action)
    fmu.getDerivatives(_pdz, nz)
    t += dt
    z += dz*dt
    fmu.setTime(t)
    fmu.setContinuousStates(_pz, nz)
    nextObservation = np.array(fmu.getReal(iO))

    termination = False
    r = 1
    if abs(nextObservation[0])>2.5 or abs(nextObservation[1])>0.2:
      termination = True
      r = -100
  
    memory.append(np.block([observation, action, nextObservation, np.array([r])]))
  
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
      steps += i
      break
  print("end of episode:",n,"steps",i,"total steps",steps)

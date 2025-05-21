#!/usr/bin/python3

import numpy as np
import torch
import torch.nn as nn
import torch.nn.functional as F
import mbsim
import mbsimControl

NS="{http://mbsimReinforcementLearning}"

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

class ActorSignal(mbsimControl.SignalSensor):
  def __init__(self):
    super(ActorSignal, self).__init__("")
    self.actorNet = Actor(4, 1, 10)

  def init(self, stage, config):
    if stage==self.resolveStringRef:
      self.actorNet.load_state_dict(torch.load(self.path+"actor_net.pt", weights_only=True))
    super(ActorSignal, self).init(stage, config)

  def getSignalSize(self): 
    return 1

  def updateSignal(self):
    super(ActorSignal, self).updateSignal()
    with torch.no_grad():
      self.s = np.array(self.actorNet(torch.tensor(self.signal.evalSignal(), dtype=torch.float32)).numpy(), dtype=np.double)
    self.upds = False

  @staticmethod
  def getSchema():
    import xml.etree.cElementTree as ET
    xsd=ET.Element(mbsim.XS+'sequence')
    xsd.append(ET.Element(mbsim.XS+'element', {'name': "neuralNetworkPath", 'type': ET.QName(mbsim.PV+"stringFullEval")}))
    return xsd

  def initializeUsingXML(self, e):
    super(ActorSignal, self).initializeUsingXML(e)
    self.path=e.find(NS+"neuralNetworkPath").text[1:-1]+"/"

mbsim.registerClass(ActorSignal)

def generateXMLSchemaFile():
  return mbsim.generateXMLSchemaFile(__name__)

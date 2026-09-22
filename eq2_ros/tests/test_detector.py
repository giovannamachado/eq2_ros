from src.eq2_ros.handDetector import *
import pytest
from random import randint,random
@pytest.fixture
def detector():
    instancia = handDetection()
    return instancia
@pytest.fixture
def handD():
    hand = {i:[randint(1,100)*random() for _ in range(4)] for i in range(5)}
    instancia = handDist(0.5,1.0,finger_dists=hand,comp_fingers={2:1.0})
    return instancia
@pytest.mark.parametrize(
   "a, esperado",
   [
       (0.5, True),
       (1, False),
   ],
   ids=["a", "b"],)
def test_handDist(handD,a,esperado):
    print(handD)
    assert (handD.x == a) == esperado

def test_Detector(detector:handDetection):
    detector.run()
    dist = detector.hand_dist
    assert dist != None
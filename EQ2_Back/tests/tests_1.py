import handDetect.handDetector
import pytest
from random import randint,random
@pytest.fixture
def detector():
    instancia = handDetector.handDetection()
    return instancia
@pytest.fixture
def handD():
    hand = {i:[randint(1,100)*random() for _ in range(4)] for i in range(5)}
    instancia = handDetector.handDist(0.5,1.0,finger_dists=hand,comp_fingers={2:1.0})
    return instancia
def test_1(handD):
    print(handD)
    assert handD.x == 0.5
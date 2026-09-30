from fs1.states import *
import pytest
state_list = [("OFF",True,"HOME"),
    ("OFF",True,"HOME"),
    ("HOME",True,"MOVING"),
    ("HOME","move","MOVING"),
    ("HOME","aaaa","HOME"),

    ("IDLE",True,"MOVING"),
    ("IDLE","home","RETURNING"),
    ("IDLE","move","MOVING"),
    ("IDLE",False,"IDLE"),
    ("IDLE","True","IDLE"),

    ("IDLE-BORDER",True,"MOVING"),
    ("IDLE-BORDER","home","RETURNING"),
    ("IDLE-BORDER","border","MOVING-BORDER"),
    ("IDLE-BORDER",False,"IDLE"),
    ("IDLE-BORDER","True","IDLE"),

    ("MOVING","normal","IDLE"),
    ("MOVING","border","IDLE-BORDER"),
    ("MOVING","stop","MOVING"),
    ("MOVING",True,"MOVING"),

    ("MOVING-BORDER","border","IDLE-BORDER"),
    ("MOVING-BORDER","stop","IDLE-BORDER"),
    ("MOVING-BORDER","normal","MOVING"),
    ("MOVING-BORDER","move","MOVING"),
    ("MOVING-BORDER",True,"MOVING-BORDER"),

    ("PRE-PICK","normal","MOVING"),
    ("PRE-PICK","move","MOVING"),
    ("PRE-PICK","pick","PICKING"),
    ("PRE-PICK","proceed","PICKING"),
    ("PRE-PICK",True,"PICKING"),
    ("PRE-PICK","aekjdofnh","PRE-PICK"),
    ("PRE-PICK",False,"PRE-PICK"),

    
    ]
lenght = max(len(str(x)) for v in state_list for x in v)
@pytest.mark.parametrize(
   "state, var, expect",state_list,
   ids=[f"{v[0]:>13}->{v[2]:<13}:{v[1]}" for v in state_list],)
def test_handDist(state,var,expect):
    r = change_state(state,var)
    assert r == expect
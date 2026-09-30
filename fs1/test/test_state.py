from fs1.states import *
import pytest
state_list = [
    ("OFF","True","HOME"),

    ("HOME","move","MOVING"),
    ("HOME","aaaa","HOME"),

    ("IDLE","home","RETURNING"),
    ("IDLE","move","MOVING"),
    ("IDLE","True","IDLE"),

    ("IDLE-BORDER","home","RETURNING"),
    ("IDLE-BORDER","border","MOVING-BORDER"),
    ("IDLE-BORDER","move","MOVING"),
    ("IDLE-BORDER","True","IDLE"),

    ("MOVING","normal","IDLE"),
    ("MOVING","border","IDLE-BORDER"),
    ("MOVING","stop","MOVING"),

    ("MOVING-BORDER","border","IDLE-BORDER"),
    ("MOVING-BORDER","normal","MOVING"),

    ("PRE-PICK","normal","MOVING"),
    ("PRE-PICK","move","MOVING"),
    ("PRE-PICK","pick","PICKING"),
    ("PRE-PICK","proceed","PICKING"),
    ("PRE-PICK","aekjdofnh","PRE-PICK"),

    
    ]
lenght = max(len(str(x)) for v in state_list for x in v)
@pytest.mark.parametrize(
   "state, var, expect",state_list,
   ids=[f"{v[0]:>13}->{v[2]:<13}:{v[1]}" for v in state_list],)
def test_handDist(state,var,expect):
    r = change_state(state,var)
    assert r == expect
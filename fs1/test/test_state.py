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
    ("IDLE-BORDER","True","IDLE-BORDER"),

    ("MOVING","normal","IDLE"),
    ("MOVING","border","IDLE-BORDER"),
    ("MOVING","stop","MOVING"),

    ("MOVING-BORDER","border","IDLE-BORDER"),
    ("MOVING-BORDER","normal","MOVING"),
    ("MOVING-BORDER","ffff","MOVING-BORDER"),

    ("PRE-PICK","move","MOVING"),
    ("PRE-PICK","pick","PICKING"),
    ("PRE-PICK","aekjdofnh","PRE-PICK"),

    ("PICKING","picked","POST-PICK"),
    ("PICKING","stop","PICKING"),

    ("POST-PICK","place","PLACING"),
    ("POST-PICK","home","RETURNING"),
    ("POST-PICK","fail","RETURNING"),
    ("POST-PICK","aaa","POST-PICK"),

    ("PLACING","sucess","RETURNING"),
    ("PLACING","aaa","PLACING"),

    ("RETURNING","sucess","HOME"),
    ("RETURNING","aaa","RETURNING"),
    ]
@pytest.mark.parametrize(
   "state, var, expect",state_list,
   ids=[f"{v[0]:>13}->{v[2]:<13}:{v[1]}" for v in state_list],)
def test_handDist(state,var,expect):
    r = change_state(state,var)
    assert r == expect
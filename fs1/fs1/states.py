
def change_state(state:str,var:str):
    match(state):
        case("OFF"): return "HOME"
        case("HOME"): 
            if    "move" == var: return "MOVING"
        case("IDLE"): 
            if    "move" == var: return "MOVING"
            elif  "home" == var: return "RETURNING"
        case("IDLE-BORDER"): 
            if    "border" == var: return "MOVING-BORDER"
            elif  "home" == var: return "RETURNING"
            elif  "move" == var: return "MOVING"
        case("MOVING"): 
            if   "normal" == var: return "IDLE"
            elif "border" == var: return "IDLE-BORDER"
        case("MOVING-BORDER"): 
            if   "border" == var: return "IDLE-BORDER"
            elif "normal" == var: return "MOVING"
        case("PRE-PICK"): 
            if   "move" == var: return "MOVING"
            elif "pick" == var: return "PICKING"
        case("PICKING"): 
            if "picked" == var: return "POST-PICK"
        case("POST-PICK"): 
            if "place" == var: return "PLACING"
            elif "home" == var or "fail" == var: return "RETURNING"
        case("PLACING"):
            if "sucess" == var: return "RETURNING"
        case("RETURNING"):
            if "sucess" == var: return "HOME"
    return state
#OFF -> HOME
#HOME -> MOVING
#MOVING <> IDLE
#MOVING -> IDLE-BORDER
#IDLE-BORDER <> MOVING-BORDER
#MOVING-BORDER -> MOVING
#MOVING <> PRE-PICK
#PRE-PICK -> PICKING
#PICKING -> POST-PICK
#POST-PICK -> PLACING
#PLACING -> RETURNING -> HOME
#POST-PICK -> RETURNING -> HOME
#IDLE -> RETURNING -> HOME
#IDLE-BORDER -> RETURNING -> HOME

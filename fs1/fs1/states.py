
state_list = ["OFF",
              "HOME","MOVING","IDLE","IDLE-BORDER","MOVING-BODER"
              "PRE-PICK","PICKING",
              "PRE-PLACE","PLACING","RETURNING"]
def change_state(state:str,var:list|bool):
    if var:
        match(state):
            case("OFF"): return "HOME"
            case("HOME"): 
                if var: return "MOVING"
            case("MOVING"): 
                return "IDLE"
            case("IDLE"): 
                return "HOME"
            case("IDLE-BORDER"): 
                return "HOME"
            case("MOVING-BODER"): 
                return "HOME"
            case("PRE-PICK"): 
                return "HOME"
            case("PICKING"): 
                return "HOME"
            case("PRE-PLACE"): 
                if any(v in var for v in ["end",True,"concluded"]): return "PLACING"
            case("PLACING"):
                if any(v in var for v in ["end",True,"concluded"]): return "RETURNING"
            case("RETURNING"):
                if any(v in var for v in ["end",True,"concluded"]): return "HOME"
#OFF -> HOME
#HOME -> MOVING
#MOVING <> IDLE
#MOVING -> IDLE-BORDER
#IDLE-BORDER <> MOVING-BODER
#MOVING-BODER -> MOVING
#MOVING <> PRE-PICK
#PRE-PICK -> PICKING
#PICKING -> PRE-PLACE
#PRE-PLACE -> PLACING
#PLACING -> RETURNING -> HOME

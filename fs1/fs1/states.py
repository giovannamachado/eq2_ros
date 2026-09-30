
def change_state(state:str,var:str|bool):
    if var:
        match(state):
            case("OFF"): return "HOME"
            case("HOME"): 
                if isinstance(var,bool) or "move" == var: return "MOVING"
            case("IDLE"): 
                if isinstance(var,bool) or "move" == var: return "MOVING"
                elif isinstance(var,str) and "home" == var: return "RETURNING"
            case("IDLE-BORDER"): 
                if isinstance(var,bool) or "move" == var: return "MOVING-BODER"
                elif isinstance(var,str) and "home" == var: return "RETURNING"
            case("MOVING"): 
                if isinstance(var,str):
                    if   "normal" == var: return "IDLE"
                    elif "border" == var: return "IDLE-BORDER"
            case("MOVING-BODER"): 
                if isinstance(var,str):
                    if   any(k == var for k in ["border","stop"]): return "IDLE-BORDER"
                    elif any(k == var for k in ["normal","move"]): return "IDLE-BORDER"
            case("PRE-PICK"): 
                if isinstance(var,str) and any(k == var for k in ["normal","move"]): return "MOVING"
                elif isinstance(var,bool) or ("pick" == var or "proceed" == var): return "PICKING"
            case("PICKING"): 
                if isinstance(var,bool) or any(k == var for k in ["picked","concluded"]): return "POST-PICK"
            case("POST-PICK"): 
                if any(v == var for v in ["end","concluded"]): return "PLACING"
                elif isinstance(var,str) and ("home" == var or "fail" == var): return "RETURNING"
            case("PLACING"):
                if any(v == var for v in ["end","concluded"]): return "RETURNING"
            case("RETURNING"):
                if any(v == var for v in ["end","concluded"]): return "HOME"
#OFF -> HOME
#HOME -> MOVING
#MOVING <> IDLE
#MOVING -> IDLE-BORDER
#IDLE-BORDER <> MOVING-BODER
#MOVING-BODER -> MOVING
#MOVING <> PRE-PICK
#PRE-PICK -> PICKING
#PICKING -> POST-PICK
#POST-PICK -> PLACING
#PLACING -> RETURNING -> HOME
#POST-PICK -> RETURNING -> HOME
#IDLE -> RETURNING -> HOME
#IDLE-BORDER -> RETURNING -> HOME

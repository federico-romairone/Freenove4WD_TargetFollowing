from . import config
from . import utility
from .models.controller import Controller

def startup() -> Controller:
    # set the reference
    ref = input("Set the target distance: ")
    if utility.is_numeric(ref) and float(ref) >= config.MIN_REF and float(ref) <= config.MAX_REF:
        ref = float(ref)
        controller = Controller(ref)
    else:
        print("Input not valid, reference initialized to default.")
        controller = Controller() # no argument --> set default
    return controller;

def main() -> int: 
    controller = None
    try:
        print("----- Freenove 4WD smart car target following -----\n")
        print("Program started, press ctrl+c to exit.\n")
        controller = startup()
        input("Put the reference in front of the sensor and press enter to start.")
        controller.follow_target();
    
    except KeyboardInterrupt:
        print("\nKeyboard interrupt triggered.")

    except Exception as e:
        print(f"\nUnkown excption triggered: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        if controller != None: controller.close()
        print("\n------------------ Program ended ------------------")
        return 0

if __name__ == "__main__":
    main()
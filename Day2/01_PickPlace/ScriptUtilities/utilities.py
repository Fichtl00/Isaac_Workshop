def log_warning(msg):
    try:
        import carb
        carb.log_warn(msg)
    except ImportError:
        print(f"WARNING: {msg}")

def log_error(msg):
    try:
        import carb
        carb.log_error(msg)
    except ImportError:
        print(f"ERROR: {msg}")

def check_kit_app_availability():
    import os
    try:
        import omni.kit.app
        try:
            app = omni.kit.app.get_app()
            if app is not None:
                log_error("Detected SimulatorApp instance. Exiting application script. (Do not run script from within App)")
                return False
        except RuntimeError:
            # This happens if the Kit app is not running (e.g., in python.sh)
            pass
    except ImportError:
        log_warning("omni.isaac.kit not found. No Omniverse-related libraries available.")
        return False
    return True
    
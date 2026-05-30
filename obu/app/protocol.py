import time


MCM_TYPE_DEFAULT = 8
MCM_ACTION_REQUEST = 1
MCM_ACTION_ACCEPT = 2
MCM_ACTION_REJECT = 3
MAX_MANOEUVRE_ID = 255

STATE_CRUISE = "CRUISE"
STATE_NEGOTIATING = "NEGOTIATING"
STATE_YIELDING = "YIELDING"
STATE_MERGING = "MERGING"
STATE_ABORT = "ABORT"


def mcm_action_name(action):
    if action == MCM_ACTION_REQUEST:
        return "REQUEST"
    if action == MCM_ACTION_ACCEPT:
        return "ACCEPT"
    if action == MCM_ACTION_REJECT:
        return "REJECT"
    return str(action)


def ms_since_minute():
    return int(time.time() * 1000) % 65536

import sys
sys.path.append('/opt/openrobots/share/example-robot-data/unittest/')
from unittest_utils import loadHyQ, loadTalos, loadTalosArm, loadTiago, loadTiagoNoHand, loadICub


DISPLAY_HYQ = 'hyq' in sys.argv
DISPLAY_TALOS = 'talos' in sys.argv
DISPLAY_TALOS_ARM = 'talos_arm' in sys.argv
DISPLAY_TIAGO = 'tiago' in sys.argv
DISPLAY_TIAGO_NO_HAND = 'tiago_no_hand' in sys.argv
DISPLAY_ICUB = 'icub' in sys.argv

if DISPLAY_HYQ:
    hyq = loadHyQ()
    hyq.initDisplay(loadModel=True)
    hyq.display(hyq.q0)

if DISPLAY_TALOS:
    talos = loadTalos()
    talos.initDisplay(loadModel=True)
    talos.display(talos.q0)

if DISPLAY_TALOS_ARM:
    talos_arm = loadTalos()
    talos_arm.initDisplay(loadModel=True)
    talos_arm.display(talos_arm.q0)

if DISPLAY_TIAGO:
    tiago = loadTiago()
    tiago.initDisplay(loadModel=True)
    tiago.display(tiago.q0)

if DISPLAY_TIAGO_NO_HAND:
    tiago_no_hand = loadTiagoNoHand()
    tiago_no_hand.initDisplay(loadModel=True)
    tiago_no_hand.display(tiago_no_hand.q0)

if DISPLAY_ICUB:
    icub = loadICub()
    icub.initDisplay(loadModel=True)
    icub.display(icub.q0)


import sys

from . import robots_loader

DISPLAY_HYQ = 'hyq' in sys.argv
DISPLAY_TALOS = 'talos' in sys.argv
DISPLAY_TALOS_ARM = 'talos_arm' in sys.argv
DISPLAY_TALOS_LEGS = 'talos_legs' in sys.argv
DISPLAY_TIAGO = 'tiago' in sys.argv
DISPLAY_TIAGO_NO_HAND = 'tiago_no_hand' in sys.argv
DISPLAY_ICUB = 'icub' in sys.argv

if DISPLAY_HYQ:
    hyq = robots_loader.loadHyQ()
    hyq.initViewer(loadModel=True)
    hyq.display(hyq.q0)

if DISPLAY_TALOS:
    talos = robots_loader.loadTalos()
    talos.initViewer(loadModel=True)
    talos.display(talos.q0)

if DISPLAY_TALOS_ARM:
    talos_arm = robots_loader.loadTalosArm()
    talos_arm.initViewer(loadModel=True)
    talos_arm.display(talos_arm.q0)

if DISPLAY_TALOS_LEGS:
    talos_legs = robots_loader.loadTalosLegs()
    talos_legs.initViewer(loadModel=True)
    talos_legs.display(talos_legs.q0)

if DISPLAY_TIAGO:
    tiago = robots_loader.loadTiago()
    tiago.initViewer(loadModel=True)
    tiago.display(tiago.q0)

if DISPLAY_TIAGO_NO_HAND:
    tiago_no_hand = robots_loader.loadTiagoNoHand()
    tiago_no_hand.initViewer(loadModel=True)
    tiago_no_hand.display(tiago_no_hand.q0)

if DISPLAY_ICUB:
    icub = robots_loader.loadICub()
    icub.initViewer(loadModel=True)
    icub.display(icub.q0)

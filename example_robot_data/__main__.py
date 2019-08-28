from argparse import ArgumentParser

from . import robots_loader

ROBOTS = ['hyq', 'talos', 'talos_arm', 'talos_legs', 'tiago', 'tiago_no_hand', 'icub']

parser = ArgumentParser()
parser.add_argument('robot', nargs='?', default=ROBOTS[0], choices=ROBOTS)

args = parser.parse_args()

if args.robot == 'hyq':
    hyq = robots_loader.loadHyQ()
    hyq.initDisplay(loadModel=True)
    hyq.display(hyq.q0)

elif args.robot == 'talos':
    talos = robots_loader.loadTalos()
    talos.initDisplay(loadModel=True)
    talos.display(talos.q0)

elif args.robot == 'talos_arm':
    talos_arm = robots_loader.loadTalosArm()
    talos_arm.initDisplay(loadModel=True)
    talos_arm.display(talos_arm.q0)

if args.robot == 'laso_legs':
    talos_legs = robots_loader.loadTalosLegs()
    talos_legs.initViewer(loadModel=True)
    talos_legs.display(talos_legs.q0)

elif args.robot == 'tiago':
    tiago = robots_loader.loadTiago()
    tiago.initDisplay(loadModel=True)
    tiago.display(tiago.q0)

elif args.robot == 'tiago_no_hand':
    tiago_no_hand = robots_loader.loadTiagoNoHand()
    tiago_no_hand.initDisplay(loadModel=True)
    tiago_no_hand.display(tiago_no_hand.q0)

elif args.robot == 'icub':
    icub = robots_loader.loadICub()
    icub.initDisplay(loadModel=True)
    icub.display(icub.q0)

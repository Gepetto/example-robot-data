from argparse import ArgumentParser

import eigenpy

from . import robots_loader

eigenpy.switchToNumpyMatrix()

ROBOTS = [
    'anymal', 'anymal_kinova', 'hyq', 'solo', 'solo12', 'talos', 'talos_arm', 'talos_legs', 'kinova', 'tiago',
    'tiago_no_hand', 'icub', 'ur5', 'romeo', 'hector', 'double_pendulum', 'iris', 'panda'
]

parser = ArgumentParser()
parser.add_argument('robot', nargs='?', default=ROBOTS[0], choices=ROBOTS)

args = parser.parse_args()

if args.robot == 'anymal':
    anymal = robots_loader.loadANYmal()
    anymal.initViewer(loadModel=True)
    anymal.display(anymal.q0)

elif args.robot == 'anymal_kinova':
    anymal = robots_loader.loadANYmal(withArm='kinova')
    anymal.initViewer(loadModel=True)
    anymal.display(anymal.q0)

elif args.robot == 'hyq':
    hyq = robots_loader.loadHyQ()
    hyq.initViewer(loadModel=True)
    hyq.display(hyq.q0)

elif args.robot == 'solo':
    solo = robots_loader.loadSolo()
    solo.initViewer(loadModel=True)
    solo.display(solo.q0)

elif args.robot == 'solo12':
    solo = robots_loader.loadSolo(False)
    solo.initViewer(loadModel=True)
    solo.display(solo.q0)

elif args.robot == 'talos':
    talos = robots_loader.loadTalos()
    talos.initViewer(loadModel=True)
    talos.display(talos.q0)

elif args.robot == 'talos_arm':
    talos_arm = robots_loader.loadTalosArm()
    talos_arm.initViewer(loadModel=True)
    talos_arm.display(talos_arm.q0)

elif args.robot == 'talos_legs':
    talos_legs = robots_loader.loadTalosLegs()
    talos_legs.initViewer(loadModel=True)
    talos_legs.display(talos_legs.q0)

elif args.robot == 'kinova':
    kinova = robots_loader.loadKinova()
    kinova.initViewer(loadModel=True)
    kinova.display(kinova.q0)

elif args.robot == 'tiago':
    tiago = robots_loader.loadTiago()
    tiago.initViewer(loadModel=True)
    tiago.display(tiago.q0)

elif args.robot == 'tiago_no_hand':
    tiago_no_hand = robots_loader.loadTiagoNoHand()
    tiago_no_hand.initViewer(loadModel=True)
    tiago_no_hand.display(tiago_no_hand.q0)

elif args.robot == 'icub':
    icub = robots_loader.loadICub()
    icub.initViewer(loadModel=True)
    icub.display(icub.q0)

elif args.robot == 'ur5':
    ur5 = robots_loader.loadUR()
    ur5.initViewer(loadModel=True)
    ur5.display(ur5.q0)

elif args.robot == 'romeo':
    romeo = robots_loader.loadRomeo()
    romeo.initViewer(loadModel=True)
    romeo.display(romeo.q0)

if args.robot == 'hector':
    hector = robots_loader.loadHector()
    hector.initViewer(loadModel=True)
    hector.display(hector.q0)

if args.robot == 'double_pendulum':
    pendulum = robots_loader.loadDoublePendulum()
    pendulum.initViewer(loadModel=True)
    pendulum.display(pendulum.q0)

if args.robot == 'iris':
    iris = robots_loader.loadIris()
    iris.initViewer(loadModel=True)
    iris.display(iris.q0)

if args.robot == 'panda':
    panda = robots_loader.loadPanda()
    panda.initViewer(loadModel=True)
    panda.display(panda.q0)

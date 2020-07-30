from argparse import ArgumentParser

from .robots_loader import ROBOTS, load

ROBOTS = sorted(ROBOTS.keys())

parser = ArgumentParser(description=load.__doc__)
parser.add_argument('robot', nargs='?', default=ROBOTS[0], choices=ROBOTS)
parser.add_argument('--no-display', action='store_false')

args = parser.parse_args()

load(args.robot, display=not args.no_display)

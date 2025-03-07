import math

def dist(pos1, pos2, plane = False):
            if plane:
                return math.sqrt((pos1.x - pos2.x)**2 +
                                 (pos1.y - pos2.y)**2)
            return math.sqrt((pos1.x - pos2.x)**2 +
                             (pos1.y - pos2.y)**2 +
                             (pos1.z - pos2.z)**2)

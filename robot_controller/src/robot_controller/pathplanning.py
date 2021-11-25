import matplotlib.pyplot as plt
import numpy as np

def plotPoints(Points, newPoints, pathSegments):
    fig = plt.figure()
    ax = fig.add_subplot()

    for point in Points:
        ax.scatter(point[0], point[1], marker="o")

    for pathSegment in pathSegments:
        p1 = pathSegment.midpoint
        p2 = pathSegment.midpoint + pathSegment.norm
        ax.plot([p1[0], p2[0]], [p1[1], p2[1]])

        ax.scatter(pathSegment.lineInterp[:, 0], pathSegment.lineInterp[:, 1])

        plt.plot(pathSegment.probePattern[:, 0], pathSegment.probePattern[:, 1])

    points = Points.tolist()
    points.append(Points[0].tolist())
    points = np.asarray(points)
    ax.plot(points[:, 0], points[:, 1], "o-")

    points = newPoints.tolist()
    points.append(newPoints[0].tolist())
    points = np.asarray(points)
    ax.plot(points[:, 0], points[:, 1], "o-")

    connectingLines = np.hstack((Points, newPoints))
    for line in connectingLines:
        ax.plot([line[0], line[2]], [line[1], line[3]], "o-")

    ax.set_xlabel('X Label')
    ax.set_ylabel('Y Label')
    ax.set_aspect('equal', 'box')

    plt.show()


def plot3Dpath(path, enableContactDetection):
    fig = plt.figure()
    ax = fig.add_subplot(projection='3d')

    for i in range(len(path) - 1):
        if enableContactDetection[i + 1]:
            color = "red"
        else:
            color = "blue"
        ax.plot(path[i:i + 2, 0], path[i:i + 2, 1], path[i:i + 2, 2], color=color)
        # plt.pause(0.5)
    ax.scatter(path[:, 0], path[:, 1], path[:, 2])
    plt.show()


def getCenter(Points):
    pArray = np.asarray(Points)
    return [np.average(pArray[:, 0]), np.average(pArray[:, 1])]


def rotateAboutOrigin(points, center, angle):
    rotPoints = []
    for point in points:
        px = np.cos(angle) * (point[0] - center[0]) - np.sin(angle) * (point[1] - center[1]) + center[0]
        py = np.sin(angle) * (point[0] - center[0]) + np.cos(angle) * (point[1] - center[1]) + center[1]
        rotPoints.append([px, py])
    return np.asarray(rotPoints)


def offsetPoints(points, offset):
    offPoints = []
    for point in points:
        offPoints.append(point + offset)
    return np.asarray(offPoints)


def getAngle(v1, v2):
    uv1 = v1 / np.linalg.norm(v1)
    uv2 = v2 / np.linalg.norm(v2)
    return np.arccos(np.dot(uv1, uv2))


def vecFromMagAngle(angle, mag=1):
    return np.asarray([mag * np.cos(angle), mag * np.sin(angle)])


def rotatePointAboutCenter(vec, point, angle):
    px = np.cos(angle) * (vec[0] - point[0]) - np.sin(angle) * (vec[1] - point[1]) + point[0]
    py = np.sin(angle) * (vec[0] - point[0]) + np.cos(angle) * (vec[1] - point[1]) + point[1]

    return np.array([px, py])


def boundSquare(points, offsetDist):
    center = getCenter(points)
    vectors = points - center
    angles = []

    offset = np.array([-3 * np.pi / 4, np.pi / 4, 5 * np.pi / 4, np.pi / 4])
    for i in range(len(points)):
        angle = getAngle(vectors[i], points[(i + 1) % 4] - points[i])

        if i % 2 == 0:
            angle -= np.pi
        angles.append(angle)

    newPoints = np.zeros((len(angles), 2))

    for i in range(len(angles)):
        newPoint = rotatePointAboutCenter(center, points[i], angles[i] + offset[i])
        newPoints[i, :] = newPoint

    newVecs = points - newPoints
    norm = np.linalg.norm(newVecs, axis=1)
    for i in range(len(angles)):
        newVecs[i, :] = newVecs[i, :] / norm[i]

    length = np.linalg.norm(np.array([offsetDist, offsetDist]))
    step = newVecs * length

    offsetSquare = points - step
    return offsetSquare


class pathSegment():
    def __init__(self, p1, p2, center, intRes, offset, penDist):
        self.p1 = p1
        self.p2 = p2
        self.lenght = np.linalg.norm(self.p2 - self.p1)

        self.midpoint = (self.p2 + self.p1) / 2
        self.norm = self.calcNorm(center)

        self.lineInterp = np.linspace(self.p1, self.p2, int(self.lenght / intRes))
        self.endOffset = int(np.ceil(offset / intRes)) + 1
        self.enableContactDetection = []
        self.probePattern = self.createProbePattern(dist=offset + penDist)

    def calcNorm(self, point):
        norm = self.midpoint - point
        return norm / np.linalg.norm(norm)

    def createProbePattern(self, dist=0.1):

        pattern = []
        for i, point in enumerate(self.lineInterp[self.endOffset - 1:-self.endOffset + 1]):
            pattern.append(point.tolist())
            self.enableContactDetection.append(False)
            if i < self.lineInterp[self.endOffset - 1:-self.endOffset + 1].shape[0] - 1:
                pattern.append((point - self.norm * dist).tolist())
                self.enableContactDetection.append(True)

        return np.asarray(pattern)


def probingPath(square, offset, resolution, penDist):
    center = getCenter(square)

    pathSegments = []
    pathSegments.append(
        pathSegment(p1=square[-1], p2=square[0], center=center, intRes=resolution, offset=offset, penDist=penDist))

    for i in range(square.shape[0] - 1):
        pathSegments.append(pathSegment(p1=square[i], p2=square[i + 1], center=center, intRes=resolution, offset=offset,
                                        penDist=penDist))

    return pathSegments


def addZandMerge(pathSegments, z):
    zPathSegments = []
    enableContactDetection = []

    for segment in pathSegments:

        for seg in segment.probePattern:
            zSeg = np.append(seg, z)
            zPathSegments.append(zSeg.tolist())

        for bool in segment.enableContactDetection:
            enableContactDetection.append(bool)

    return np.asarray(zPathSegments), np.asarray(enableContactDetection)


class sidePathPlaner():
    def __init__(self, square, offset=0.05, resolution=0.01, penDist=0.0, toolOffset=[0, 0, 0.05]):
        self.square = square
        self.offsetSquare = boundSquare(square[:, 0:2], offset)

        self.pathSegments2D = probingPath(self.offsetSquare, offset, resolution, penDist)
        self.path3D, self.enableContactDetection = addZandMerge(self.pathSegments2D, np.average(square[:, 2]))
        self.path3D += toolOffset
        self.path3D = np.insert(self.path3D, 0, self.path3D[0] + [0, 0, 0.2], axis=0)
        self.path3D = np.append(self.path3D, [self.path3D[-1] + [0, 0, 0.2]], axis=0)

        self.enableContactDetection = np.insert(self.enableContactDetection, 0, False, axis=0)
        self.enableContactDetection = np.append(self.enableContactDetection, [False], axis=0)

    def getPath(self):
        return self.path3D

    def plot2D(self):
        plotPoints(self.square[:, 0:2], self.offsetSquare, self.pathSegments2D)

    def plot3D(self):
        plot3Dpath(self.path3D, self.enableContactDetection)


class line3D():
    def __init__(self, p1, p2, intRes):
        self.p1 = p1
        self.p2 = p2
        self.lenght = np.linalg.norm(self.p2 - self.p1)
        self.linInterp = np.linspace(self.p1, self.p2, int(self.lenght / intRes))

    def getLinInterp(self):
        return self.linInterp


class probePathTop():
    def __init__(self, square, zOffset=0.1, intRes=[0.1, 0.1], penDist=0.01, toolOffset=0.1):
        self.intRes = intRes
        self.zOffset = zOffset
        self.toolOffset = toolOffset
        self.penDist = penDist
        self.square = square + [0, 0, zOffset + toolOffset]
        self.opLines = [line3D(self.square[0], self.square[1], intRes=intRes[0]),
                        line3D(self.square[3], self.square[2], intRes=intRes[0])]
        self.enableContactDetection = []
        self.connectingLines = self.makeConnectingLines()
        self.path = self.makePathFromConnectingLines()

    def makeConnectingLines(self):
        connectinLines = []
        for i in range(np.minimum(self.opLines[0].linInterp.shape[0], self.opLines[1].linInterp.shape[0])):
            connectinLines.append(
                line3D(self.opLines[0].linInterp[i], self.opLines[1].linInterp[i], intRes=self.intRes[1]))
        return connectinLines

    def makePathFromConnectingLines(self):

        pathSegments = []
        for i in range(0, len(self.connectingLines) - 1, 2):
            pathSegments.append(self.connectingLines[i].linInterp)
            pathSegments.append(np.flip(self.connectingLines[i + 1].linInterp, axis=0))

        path = pathSegments[0]
        for segment in pathSegments[1:]:
            path = np.append(path, segment, axis=0)

        fullPath = []
        for point in path:
            fullPath.append(point)
            self.enableContactDetection.append(False)
            fullPath.append(point - [0, 0, self.zOffset + self.penDist])
            self.enableContactDetection.append(True)
            fullPath.append(point)
            self.enableContactDetection.append(False)

        return np.asarray(fullPath)

    def plot(self):
        plot3Dpath(self.path, self.enableContactDetection)

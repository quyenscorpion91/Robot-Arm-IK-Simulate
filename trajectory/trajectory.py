import math


class Trajectory:
    def __init__(self):
        print("init")
    
    # use Bresenham algorithm (we can also use DDA instead)
    def BresenhamLinear(x1, y1, z1, x2, y2, z2):
        class Point_3D:
            x = []
            y = []
            z = []
        ListOfPoints = Point_3D
        ListOfPoints.x.append(x1)
        ListOfPoints.y.append(y1)
        ListOfPoints.z.append(z1)
        dx = abs(x2 - x1)
        dy = abs(y2 - y1)
        dz = abs(z2 - z1)
        if (x2 > x1):
            xs = 1
        else:
            xs = -1
        if (y2 > y1):
            ys = 1
        else:
            ys = -1
        if (z2 > z1):
            zs = 1
        else:
            zs = -1

        # x-axis
        if (dx >= dy and dx >= dz):        
            p1 = 2 * dy - dx
            p2 = 2 * dz - dx
            while (x1 != x2):
                x1 += xs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dx
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dx
                p1 += 2 * dy
                p2 += 2 * dz
                ListOfPoints.x.append(x1)
                ListOfPoints.y.append(y1)
                ListOfPoints.z.append(z1)

        # y-axis
        elif (dy >= dx and dy >= dz):       
            p1 = 2 * dx - dy
            p2 = 2 * dz - dy
            while (y1 != y2):
                y1 += ys
                if (p1 >= 0):
                    x1 += xs
                    p1 -= 2 * dy
                if (p2 >= 0):
                    z1 += zs
                    p2 -= 2 * dy
                p1 += 2 * dx
                p2 += 2 * dz
                ListOfPoints.x.append(x1)
                ListOfPoints.y.append(y1)
                ListOfPoints.z.append(z1)
        # z-axis
        else:        
            p1 = 2 * dy - dz
            p2 = 2 * dx - dz
            while (z1 != z2):
                z1 += zs
                if (p1 >= 0):
                    y1 += ys
                    p1 -= 2 * dz
                if (p2 >= 0):
                    x1 += xs
                    p2 -= 2 * dz
                p1 += 2 * dy
                p2 += 2 * dx
                ListOfPoints.x.append(x1)
                ListOfPoints.y.append(y1)
                ListOfPoints.z.append(z1)

        return ListOfPoints

    def DDACircle(x0, y0, z0, xc, yc, r):
        class Point_3D:
            x = []
            y = []
            z = []
        ListOfPoints = Point_3D
        Q1 = 0
        Q2 = 0
        P1 = xc
        P2 = yc
        x = x0
        y = y0
        z = z0
        # just for test, generate for a clockwise direction in the first quadrant
        perimeter = (math.pi * 2 * r) /  4
        BLU = 1
        for i in range(int(perimeter + 1)):
            Q1 += P1
            if Q1 >= r:
                Q1 = Q1 - r
                P2 = P2 + 1
                y += 1
                ListOfPoints.x.append(x)
                ListOfPoints.y.append(y)
                ListOfPoints.z.append(z)
            Q2 += P2
            if Q2 >= r:
                Q2 = Q2 - r
                P1 = P1 - 1
                x += 1
                ListOfPoints.x.append(x)
                ListOfPoints.y.append(y)
                ListOfPoints.z.append(z)

        return ListOfPoints

    # this just draw 1/8 given arc
    def BresenhamCircle(xc, yc, z,  r, arc):
        class Point_3D:
            x = []
            y = []
            z = []
        x = 0
        y = r
        d = 3 - 2 * r
        ListOfPoints = Point_3D
        if arc == 0:
            ListOfPoints.x.append(xc + x)
            ListOfPoints.y.append(yc + y)
            ListOfPoints.z.append(z)
        elif arc == 1:
            ListOfPoints.x.append(xc + y)
            ListOfPoints.y.append(yc + x)
            ListOfPoints.z.append(z)
        while y > x:
            x += 1
            if d > 0:
                y -= 1
                d = d + 4 * (x - y) + 10
            else :
                d = d + 4 * x + 6
            if arc == 0:
                ListOfPoints.x.append(xc + x)
                ListOfPoints.y.append(yc + y)
                ListOfPoints.z.append(z)
            elif arc == 1:
                ListOfPoints.x.append(xc + y)
                ListOfPoints.y.append(yc + x)
                ListOfPoints.z.append(z)


        return ListOfPoints



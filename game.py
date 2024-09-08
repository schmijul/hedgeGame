from direct.showbase.ShowBase import ShowBase
from panda3d.core import Point3, Vec3, Vec4, AmbientLight, DirectionalLight
from panda3d.core import GeomNode, Geom, GeomVertexFormat, GeomVertexData
from panda3d.core import GeomTriangles, GeomVertexWriter
from panda3d.core import CollisionNode, CollisionSphere, CollisionBox
from panda3d.core import CollisionTraverser, CollisionHandlerPusher
from direct.task import Task
from math import sin, cos, pi
import random

class HedgehogGame(ShowBase):
    def __init__(self):
        ShowBase.__init__(self)

        self.disableMouse()
        self.camera.setPos(5, -10, 5)
        self.camera.lookAt(Point3(0, 0, 0))

        # Create a simple sphere for the hedgehog
        self.hedgehog = self.create_sphere(0.5, 16, 16)
        self.hedgehog.setColor(0.8, 0.6, 0.4)
        self.hedgehog.setPos(0, 0, 0.5)

        # Add collision sphere to hedgehog
        coll_sphere = CollisionNode('hedgehog_collision')
        coll_sphere.addSolid(CollisionSphere(0, 0, 0, 0.5))
        self.hedgehog_coll = self.hedgehog.attachNewNode(coll_sphere)

        # Create a larger plane for the ground
        self.ground = self.create_plane(50, 50)
        self.ground.setColor(0.5, 0.7, 0.5)
        self.ground.setPos(0, 0, 0)

        # Create multiple cube obstacles
        self.obstacles = []
        for _ in range(10):
            obstacle = self.create_cube(1)
            obstacle.setColor(0.7, 0.3, 0.3)
            x = random.uniform(-20, 20)
            y = random.uniform(-20, 20)
            obstacle.setPos(x, y, 0.5)
            
            # Add collision box to obstacle
            coll_box = CollisionNode('obstacle_collision')
            coll_box.addSolid(CollisionBox(Point3(0, 0, 0), 0.5, 0.5, 0.5))
            obstacle_coll = obstacle.attachNewNode(coll_box)
            
            self.obstacles.append(obstacle)

        # Set up lighting
        alight = AmbientLight('alight')
        alight.setColor(Vec4(0.2, 0.2, 0.2, 1))
        alnp = self.render.attachNewNode(alight)
        self.render.setLight(alnp)

        dlight = DirectionalLight('dlight')
        dlight.setDirection(Vec3(-5, -5, -5))
        dlight.setColor(Vec4(0.8, 0.8, 0.8, 1))
        dlnp = self.render.attachNewNode(dlight)
        self.render.setLight(dlnp)

        # Set up key event handlers
        self.keys = {"left": 0, "right": 0, "up": 0, "down": 0, "jump": 0}
        self.accept("arrow_left", self.updateKeyMap, ["left", 1])
        self.accept("arrow_left-up", self.updateKeyMap, ["left", 0])
        self.accept("arrow_right", self.updateKeyMap, ["right", 1])
        self.accept("arrow_right-up", self.updateKeyMap, ["right", 0])
        self.accept("arrow_up", self.updateKeyMap, ["up", 1])
        self.accept("arrow_up-up", self.updateKeyMap, ["up", 0])
        self.accept("arrow_down", self.updateKeyMap, ["down", 1])
        self.accept("arrow_down-up", self.updateKeyMap, ["down", 0])
        self.accept("space", self.updateKeyMap, ["jump", 1])
        self.accept("space-up", self.updateKeyMap, ["jump", 0])

        self.velocity = Vec3(0, 0, 0)
        self.gravity = -9.8
        self.jumpStrength = 5

        # Set up collision traverser and handler
        self.cTrav = CollisionTraverser()
        self.collHandler = CollisionHandlerPusher()
        self.collHandler.addCollider(self.hedgehog_coll, self.hedgehog)
        self.cTrav.addCollider(self.hedgehog_coll, self.collHandler)

        self.taskMgr.add(self.update, "update")

    def create_sphere(self, radius, lat_segments, lon_segments):
        format = GeomVertexFormat.getV3n3c4()
        vdata = GeomVertexData('sphere', format, Geom.UHStatic)
        
        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')
        color = GeomVertexWriter(vdata, 'color')

        for i in range(lat_segments + 1):
            for j in range(lon_segments + 1):
                theta = i * pi / lat_segments
                phi = j * 2 * pi / lon_segments
                x = radius * sin(theta) * cos(phi)
                y = radius * sin(theta) * sin(phi)
                z = radius * cos(theta)
                vertex.addData3(x, y, z)
                normal.addData3(x, y, z)
                color.addData4(1, 1, 1, 1)

        prim = GeomTriangles(Geom.UHStatic)

        for i in range(lat_segments):
            for j in range(lon_segments):
                prim.addVertices(
                    i * (lon_segments + 1) + j,
                    i * (lon_segments + 1) + j + 1,
                    (i + 1) * (lon_segments + 1) + j)
                prim.addVertices(
                    i * (lon_segments + 1) + j + 1,
                    (i + 1) * (lon_segments + 1) + j + 1,
                    (i + 1) * (lon_segments + 1) + j)

        geom = Geom(vdata)
        geom.addPrimitive(prim)
        node = GeomNode('sphere')
        node.addGeom(geom)
        return self.render.attachNewNode(node)

    def create_plane(self, width, height):
        format = GeomVertexFormat.getV3n3c4()
        vdata = GeomVertexData('plane', format, Geom.UHStatic)
        
        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')
        color = GeomVertexWriter(vdata, 'color')

        # Define the four corners of the plane
        vertex.addData3(-width/2, -height/2, 0)
        vertex.addData3(width/2, -height/2, 0)
        vertex.addData3(width/2, height/2, 0)
        vertex.addData3(-width/2, height/2, 0)

        for i in range(4):
            normal.addData3(0, 0, 1)
            color.addData4(1, 1, 1, 1)

        # Create triangles
        tri = GeomTriangles(Geom.UHStatic)
        tri.addVertices(0, 1, 2)
        tri.addVertices(0, 2, 3)

        geom = Geom(vdata)
        geom.addPrimitive(tri)
        node = GeomNode('plane')
        node.addGeom(geom)
        return self.render.attachNewNode(node)

    def create_cube(self, size):
        format = GeomVertexFormat.getV3n3c4()
        vdata = GeomVertexData('cube', format, Geom.UHStatic)
        
        vertex = GeomVertexWriter(vdata, 'vertex')
        normal = GeomVertexWriter(vdata, 'normal')
        color = GeomVertexWriter(vdata, 'color')

        # Define the eight corners of the cube
        s = size / 2
        for point in [(-s,-s,-s), (s,-s,-s), (s,s,-s), (-s,s,-s),
                      (-s,-s,s), (s,-s,s), (s,s,s), (-s,s,s)]:
            vertex.addData3f(point)
            normal.addData3f(Vec3(*point).normalize())
            color.addData4f(1, 1, 1, 1)

        # Define the six faces of the cube
        geom = Geom(vdata)
        for face in [(0,1,2,3), (4,5,6,7), (0,4,7,3),
                     (1,5,6,2), (0,1,5,4), (3,2,6,7)]:
            tri = GeomTriangles(Geom.UHStatic)
            tri.addVertices(face[0], face[1], face[2])
            tri.addVertices(face[0], face[2], face[3])
            geom.addPrimitive(tri)

        node = GeomNode('cube')
        node.addGeom(geom)
        return self.render.attachNewNode(node)

    def updateKeyMap(self, key, value):
        self.keys[key] = value

    def update(self, task):
        dt = globalClock.getDt()

        # Apply gravity
        self.velocity.z += self.gravity * dt

        # Apply movement based on key presses
        if self.keys["left"]:
            self.velocity.x = -5
        elif self.keys["right"]:
            self.velocity.x = 5
        else:
            self.velocity.x = 0

        if self.keys["up"]:
            self.velocity.y = 5
        elif self.keys["down"]:
            self.velocity.y = -5
        else:
            self.velocity.y = 0

        # Apply jumping
        if self.keys["jump"] and self.hedgehog.getZ() <= 0.5:
            self.velocity.z = self.jumpStrength

        # Update position
        pos = self.hedgehog.getPos()
        pos += self.velocity * dt
        pos.z = max(0.5, pos.z)  # Ensure hedgehog doesn't go below ground
        self.hedgehog.setPos(pos)

        # Update camera to follow hedgehog
        self.camera.setPos(pos + Vec3(10, -20, 10))
        self.camera.lookAt(pos)

        # Perform collision detection
        self.cTrav.traverse(self.render)

        return Task.cont

game = HedgehogGame()
game.run()
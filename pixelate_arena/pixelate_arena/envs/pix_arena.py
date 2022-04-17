import gym
import pybullet as p
import pybullet_data
import cv2
import numpy as np
import random

class PixelateArena(gym.Env):
    metadata = {'render.modes': ['human']}

    def __init__(self, sim = True):
        
        if sim:
            p.connect(p.GUI)
            p.setAdditionalSearchPath(pybullet_data.getDataPath())
            p.setGravity(0,0,-10)
            p.loadURDF('rsc/plane.urdf',[0,0,-0.1], useFixedBase=1, globalScaling= 3)
            p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
            p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

            self.husky = None
            self._load_arena()
            self.respawn_car()

        self._width = 720
        self._height = 720
    
    def reset_arena(self):
        """
        Function to restart the simulation.

        This will undo all the previous simulation commands and the 
        arena along with the robot will be loaded again.

        Only for testing purposes. Won't be used in final evaluation.

        Arguments:

            None

        Return Values:

            None
        """
        np.random.seed(0)
        p.resetSimulation()
        p.setGravity(0,0,-10)

        p.loadURDF('rsc/plane.urdf',[0,0,-0.1], useFixedBase=1, globalScaling= 3)
        p.configureDebugVisualizer(p.COV_ENABLE_WIREFRAME, 0)
        p.configureDebugVisualizer(p.COV_ENABLE_SHADOWS, 0)

        self._load_arena()
        self.respawn_car()

    def _load_arena(self):
        """ Function to load the arena """
        
        
        def tile_coordinates(x,y):                                  # Input : Index in x,y order
            return  [1.02*(y-6), 0.585*(x-12), -0.08]               # returns co-ordinate of tile in x,y order 
        
        self.arena = np.array(
           [[0, 0, 0, 0, 0, 0, 4, 0, 4, 0, 3, 0, 4, 0, 1, 0, 2, 0, 3, 0, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 4, 0, 2, 0, 2, 0, 0, 0, 0, 0, 1, 0, 1, 0, 1, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 4, 0, 0, 0, 5, 0, 0, 0, 0, 0, 2, 0, 2, 0, 3, 0, 1, 0, 0, 0, 0],
            [0, 0, 0, 3, 0, 2, 0, 0, 0, 0, 0, 4, 0, 2, 0, 4, 0, 4, 0, 3, 0, 1, 0, 0, 0],
            [0, 0, 4, 0, 1, 0, 1, 0, 1, 0, 7, 0, 3, 0, 4, 0, 2, 0, 3, 0, 4, 0, 4, 0, 0],
            [0, 3, 0, 2, 0, 2, 0, 1, 0, 2, 0, 2, 0, 2, 0, 2, 0, 3, 0, 2, 0, 3, 0, 3, 0],
            [1, 0, 1, 0, 3, 0, 2, 0, 1, 0, 3, 0, 5, 0, 3, 0, 7, 0, 0, 0, 4, 0, 3, 0, 3],
            [0, 2, 0, 1, 0, 4, 0, 4, 0, 0, 0, 4, 0, 3, 0, 1, 0, 0, 0, 0, 0, 3, 0, 2, 0],
            [0, 0, 1, 0, 4, 0, 0, 0, 0, 0, 7, 0, 2, 0, 4, 0, 1, 0, 0, 0, 2, 0, 3, 0, 0],
            [0, 0, 0, 1, 0, 3, 0, 0, 0, 0, 0, 1, 0, 2, 0, 3, 0, 1, 0, 0, 0, 2, 0, 0, 0],
            [0, 0, 0, 0, 2, 0, 2, 0, 4, 0, 0, 0, 1, 0, 1, 0, 5, 0, 0, 0, 4, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 3, 0, 2, 0, 0, 0, 3, 0, 2, 0, 1, 0, 2, 0, 3, 0, 0, 0, 0, 0],
            [0, 0, 0, 0, 0, 0, 2, 0, 2, 0, 1, 0, 4, 0, 3, 0, 3, 0, 4, 0, 0, 0, 0, 0, 0]]
        )
        self.villain_coords = [[31, (6 , 12)], 
                               [34, (6 , 0)], 
                               [33, (24, 6)]]

        self.color_code_urdf = dict({
            2 :  'rsc/hexagon/hexagon_yellow.urdf',
            3 :  'rsc/hexagon/hexagon_purple.urdf',
            4 :  'rsc/hexagon/hexagon_green.urdf' ,
            7 :  'rsc/hexagon/hexagon_pink.urdf'  ,
            1 :  'rsc/hexagon/hexagon_white.urdf' ,
            5 :  'rsc/hexagon/hexagon_red.urdf'   ,
            31 : 'rsc/circle/circle.urdf'         ,
            33 : 'rsc/triangle/triangle.urdf'     ,
            34 : 'rsc/square/square.urdf'
        })

        for y in range(13):
            for x in range(25):
                if self.arena[y][x] != 0:
                    p.loadURDF(self.color_code_urdf[self.arena[y][x]],tile_coordinates(x,y),useFixedBase = 1,globalScaling = 0.027)

        for v in self.villain_coords:
            p.loadURDF(self.color_code_urdf[v[0]],tile_coordinates(*v[1]),useFixedBase = 1)

    def respawn_car(self):

        """
		Function to respawn the car from the arena.

		Arguments:

			None

		Return Values:

			None
		"""

        if self.husky is not None:
            print("Old Car being Removed")
            p.removeBody(self.husky)
            self.husky = None

        pos = [0,0]
        ori = [np.pi/2, 0, np.pi/2, np.pi]
        x = np.random.randint(0, len(ori))
        self.husky = p.loadURDF('rsc/car/car.urdf', [pos[0],pos[1],0], p.getQuaternionFromEuler([0,0,ori[x]]) , globalScaling = 0.7)
        for x in range(100):
            p.stepSimulation()

    def remove_car(self):
        """
        Function to remove the car from the arena.

        Arguments:

            None

        Return Values:

            None
        """
        p.removeBody(self.husky)
        self.husky = None

    def camera_feed(self, is_flat = False):
        """
        Function to get camera feed of the arena.

        Arguments:

            None

        Return Values:

            numpy array of RGB values
        """
        look = [0, 0, 0.2]
        cameraeyepos = [0, 0, 8]
        cameraup = [0, -1, 0]
        self._view_matrix = p.computeViewMatrix(cameraeyepos, look, cameraup)
        fov = 90
        aspect = self._width / self._height
        near = 0.8
        far = 10
        self._proj_matrix = p.computeProjectionMatrixFOV(fov, aspect, near, far)
        img_arr = p.getCameraImage(width=self._width,
                                height=self._height,
                                viewMatrix=self._view_matrix,
                                projectionMatrix=self._proj_matrix,
                                renderer=p.ER_BULLET_HARDWARE_OPENGL)
        rgb = img_arr[2]
        if is_flat == True:
            # Only for those who are getting a blank image in opencv
            rgb = np.array(rgb)
            rgb = np.reshape(rgb, (720, 720, 4))
        rgb = np.uint8(rgb)
        rgb = cv2.cvtColor(rgb, cv2.COLOR_BGR2RGB)
        rgb = np.rot90(rgb, 3)
        return rgb

    def move_husky(self, leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel):
        """
        Function to give Velocities to the wheels of the robot.
            
        Arguments:

            leftFrontWheel  - Velocity of the front left wheel  
            rightFrontWheel - Velocity of the front right wheel  
            leftRearWheel   - Velocity of the rear left wheel  
            rightRearWheel  - Velocity of the rear right wheel  


        Return Values:

            None
        """

        self.__move(self.husky, leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel)

    def __move(self, car, leftFrontWheel, rightFrontWheel, leftRearWheel, rightRearWheel):
        p.setJointMotorControl2(car,  4, p.VELOCITY_CONTROL, targetVelocity = leftFrontWheel , force=30)
        p.setJointMotorControl2(car,  5, p.VELOCITY_CONTROL, targetVelocity = rightFrontWheel, force=30)
        p.setJointMotorControl2(car,  6, p.VELOCITY_CONTROL, targetVelocity = leftRearWheel  , force=30)
        p.setJointMotorControl2(car,  7, p.VELOCITY_CONTROL, targetVelocity = rightRearWheel , force=30)

    def unlock_antidotes(self):
        """
        Function to unlock the antidotes 

        Arguments:
            
            None

        Return Values:

            None    
        """
        def tile_coordinates(x,y):                                  # Input : Index in x,y order
            return  [1.02*(y-6), 0.585*(x-12), -0.08]               # returns co-ordinate of tile in x,y order 
       
        self.antidote_cords = [[34, (16,6)], 
                               [31, (10,8)], 
                               [33, (10,4)]]

        for a in self.antidote_cords:
            p.loadURDF(self.color_code_urdf[a[0]],tile_coordinates(*a[1]),useFixedBase = 1)

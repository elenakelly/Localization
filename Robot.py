from re import I
import pygame
import numpy as np
import math
import random
import Filter
import time

# initialisation of game
pygame.font.init()

# images
BACKGROUND = pygame.transform.scale(
    pygame.image.load("images/background.png"), (600, 800))
ROBOT = pygame.image.load("images/vacuum.png")
ICON = pygame.image.load('images/icon.png')
DUST = pygame.image.load('images/dust.png')

# main sceen
WIDTH, HEIGHT = 600, 800  # dimensions
SCREEN = pygame.display.set_mode((WIDTH, HEIGHT))
# window setting
pygame.display.set_caption("Mobile Robot Simulator")
pygame.display.set_icon(ICON)
pygame.rect.Rect
MAIN_FONT = pygame.font.SysFont("comicsans", 22)
SENSORS_FONT = pygame.font.SysFont("comicsans", 12)
# SENSORS - Divide circumference by number of sensors
CAST_RAYS = 24
STEP_ANGLE = (math.pi*2) / CAST_RAYS

# image visual rotation


def blit_rotate_center(win, image, top_left, angle):
    rotated_image = pygame.transform.rotozoom(image, angle, 1)
    new_rect = rotated_image.get_rect(
        center=image.get_rect(topleft=top_left).center)
    win.blit(rotated_image, new_rect.topleft)

# Robot movement


class RobotMove:
    def __init__(self):
        self.img = self.IMG  # image
        self.x = self.START_POS[0]  # starting x
        self.y = self.START_POS[1]  # starting y

        self.v = 0  # translated velocity
        self.w = 0  # angular velocitty
        self.speed = 1
        self.side = 0.01
        self.theta = 0
        # self.theta = -math.pi/2
        self.sensor_limit = 200

        distance = 64
        # distance between the centers of the two wheels
        self.l = int(ROBOT.get_width())
        self.changeX = self.x + (self.l/2)
        self.changeY = self.y

        self.rect = pygame.Rect(
            self.x, self.y, ROBOT.get_width(), ROBOT.get_height())
        
        #localization stuff
        self.movement_error = 0.1
        self.believe_states = [[self.x, self.y, self.theta]]

    # draw and rotate the image

    def draw(self, win):
        blit_rotate_center(win, self.img, (self.x, self.y),
                           math.degrees(-self.theta))

    def move(self, keys, dt):

        # setting the buttons PYGAME ADJUSTMENT left=right velocities
        if keys[0] == 1:
            self.v += self.speed
        if keys[1] == 1:
            self.v -= self.speed
        if keys[2] == 1:
            self.w -= self.side
        if keys[3] == 1:
            self.w += self.side
        if keys[4] == 1:
            self.v = 0
            self.w = 0

        # next_x, next_y = self.x, self.y

        # check model
        if self.v != 0 or self.w != 0:

            # Computation of ICC
            centerx = self.x+(ROBOT.get_width()/2)
            centery = self.y+(ROBOT.get_height()/2)

            a = [self.x, self.y, self.theta]
            b = [[dt*math.cos(self.theta), 0],
                 [dt*math.sin(self.theta), 0],
                 [0, dt]]
            c = [self.v, self.w]
            rotation = np.dot(b, c)
            M = a + rotation

            # TEMP DRAW LINE TOWARDS ICC
            pygame.draw.line(SCREEN, (255, 255, 0), (centerx, centery),
                             (M[0], M[1]), 3)

            # DRAW LINE MOVE VECTOR
            pygame.display.flip()

            # next_x = M[0]-(ROBOT.get_width()/2)
            # next_y = M[1]-(ROBOT.get_height()/2)
            next_x = M[0]
            next_y = M[1]
            # print("-")
            new_theta = M[2]

            self.x = next_x
            self.y = next_y
            self.theta = new_theta

        self.rotated = pygame.transform.rotozoom(
            self.img, math.degrees(self.theta), 1)
    
    def movement_noise(self):
        if random.random() < self.movement_error:
            if (self.x > 0.01 or self.y > 0.01):
                self.x += (random.random() - 0.5) * 0.01
            elif self.x > 0.01:
                self.x += (random.random() - 0.5) * 0.01

        if random.random() < self.movement_error:
            if (self.x < -0.01 or self.y  < -0.01):
                self.y  += (random.random() - 0.5) * 0.01
            elif self.x < -0.01:
                self.y  += (random.random() - 0.5) * 0.01
    
    def upd_rect(self):
        self.rect.x = self.x
        self.rect.y = self.y

# -------------------------------------------------------------------------------
# Raycasting


def find_beacon(screen, beacons):

    sensor_x = player_robot.x+(ROBOT.get_width()/2)
    sensor_y = player_robot.y+(ROBOT.get_height()/2)

    beacon_lines = []
    collision_offset = 32

    for bc in range(len(beacons)):
        if int(math.sqrt((beacons[bc].y-sensor_y)**2 + (beacons[bc].x-sensor_x)**2))-collision_offset < 150:
            beacon_lines.append(pygame.draw.line(screen, (255, 130, 100),
                                                 (sensor_x, sensor_y), (beacons[bc].x, beacons[bc].y), 3))


def cast_rays(screen, beacons):

    all_sensors = []
    sensor_results = []

    sensor_x = player_robot.x+(ROBOT.get_width()/2)
    sensor_y = player_robot.y+(ROBOT.get_height()/2)

    temp_angle = 0

    sensor_placement_offset = 8
    sensor_placement_radius_depth = 64
    collision_offset = 32

    detected = []
    beacon_found = False

    for i in range(CAST_RAYS):

        clipped_line = None

        for depth in range(200):
            target_x = sensor_x - math.sin(temp_angle) * depth
            target_y = sensor_y + math.cos(temp_angle) * depth

            ray = ((sensor_x, sensor_y), (target_x, target_y))

            for j in range(len(beacons)):
                clipped_line = beacons[j].rect.clipline(ray)

                for k in detected:
                    if k[1] == beacons[j].id:
                        beacon_found = True
                        break
                if beacon_found:
                    beacon_found = False
                    continue

                if clipped_line:
                    detected.append((clipped_line, beacons[j].id))
                    break

            if clipped_line:
                break

        sensor_distance = 200
        if clipped_line:
            temp_sensor_distance = int(
                math.sqrt((clipped_line[0][1]-sensor_y)**2 + (clipped_line[0][0]-sensor_x)**2))-collision_offset
            if temp_sensor_distance < sensor_distance:
                sensor_distance = temp_sensor_distance

            pygame.draw.line(screen, (255, 130, 100), (sensor_x, sensor_y),
                             (clipped_line[0][0], clipped_line[0][1]), 3)
        sensor_results.append(sensor_distance)
        # sensor_text = SENSORS_FONT.render(
        #     f"{sensor_distance}", 1, (255, 255, 255))
        # screen.blit(
        #     sensor_text, (sensor_placement_x, sensor_placement_y))

        temp_angle += STEP_ANGLE

    return sensor_results
    # ------------


# -------------------------------------------------------------------------------------

class Wall():
    def __init__(self, x, y, width, height, transparency):
        self.rect = pygame.Rect(x, y, width, height)
        self.istransparent = transparency

    def draw(self, screen):
        if not self.istransparent:
            pygame.draw.rect(screen, (49, 60, 60), self.rect)


class Beacon():
    def __init__(self, x, y, radius, screen, id):
        self.rect = pygame.draw.circle(screen, (0, 0, 0), (x, y), radius)
        self.radius = radius
        self.x = x
        self.y = y
        self.id = id

    def draw(self, screen):
        pygame.draw.circle(screen, (0, 0, 0), (self.x, self.y), self.radius)


class Envir:
    def __init__(self, dimension):
        # colors
        self.black = (0, 0, 0)
        self.white = (255, 255, 255)
        self.gray = (49, 60, 60)
        self.blue = (20, 80, 155)
        self.red = (255, 0, 0)
        # map_dims
        self.height = dimension[0]
        self.width = dimension[1]
        # window setting
        self.map = pygame.display.set_mode((self.width, self.height))
        # trails
        self.trail_set = []
        self.dash_trail_set = []

    # line route
    def trail(self, pos):
        for i in range(0, len(self.trail_set)-1):
            pygame.draw.line(self.map, self.white, (self.trail_set[i][0], self.trail_set[i][1]),
                             (self.trail_set[i+1][0], self.trail_set[i+1][1]))
        if self.trail_set.__sizeof__() > 10000:
            self.trail_set.pop(0)
        self.trail_set.append(pos)

    # estimated line route
    def dotted_line(self, pos):
        for i in range(0, len(self.dash_trail_set)-1):
            pygame.draw.line(self.map, self.black, (self.dash_trail_set[i][0]+5, self.dash_trail_set[i][1]+5),
                             (self.dash_trail_set[i+1][0]+1, self.dash_trail_set[i+1][1]+1))
        if self.dash_trail_set.__sizeof__() > 1000000:
            self.dash_trail_set.pop(0)
        self.dash_trail_set.append(pos)

    # y and x axis
    def robot_frame(self, pos, rotation):
        n = 80
        # centerx, centery = pos
        centerx = pos[0]+(ROBOT.get_width()/2)
        centery = pos[1]+(ROBOT.get_height()/2)
        x_axis = (centerx + n*np.cos(rotation), centery + n*np.sin(rotation))
        y_axis = (centerx + n*np.cos(rotation+np.pi/2),
                  centery + n*np.sin(rotation+np.pi/2))
        pygame.draw.line(self.map, self.black, (centerx, centery), x_axis, 3)
        pygame.draw.line(self.map, self.black, (centerx, centery), y_axis, 3)

    def draw(self, screen, images, player_robot):

        # display images on screen
        for img, pos, name in images:
            screen.blit(img, pos)

        # display left, right velocity and theta on screen
        vel_text = MAIN_FONT.render(
            f"v = {round(player_robot.v,2)} w = {round(player_robot.w,2)} theta = {int(np.degrees(player_robot.theta))}", 1, self.white)
        screen.blit(vel_text, (10, HEIGHT - vel_text.get_height() - 40))
        # display robot on screen
        player_robot.draw(screen)
        # pygame.display.update()

        # display beacons on the walls
        wall_list2 = [(30, 170), (400, 170), (170, 330), (570, 330), (170, 580),
                      (350, 500), (350, 750), (32, 746), (32, 54), (568, 54), (568, 746)]
        for beacons in wall_list2:
            pygame.draw.circle(SCREEN, (0, 0, 0), beacons, 7)

    def draw_elipses(self):
        x, y = player_robot.x, player_robot.y
        width, height = 2 * x / 10, 2 * y / 10
        surface = pygame.Surface((width, height))
        size = (0, 0, width, height)
        # drawing an ellipse
        pygame.draw.ellipse(surface, self.blue, size)
        rotate = [int(x - width / 2), int(y - height / 2)]
        SCREEN.blit(surface, rotate)

    def setWalls():
        wall_pixel_offset = 42
        rectWallL = pygame.Rect(0, 0, wall_pixel_offset, HEIGHT)
        rectWallR = pygame.Rect(WIDTH-wall_pixel_offset, 0,
                                wall_pixel_offset, HEIGHT)
        rectWallT = pygame.Rect(0,  0, WIDTH, wall_pixel_offset)
        rectWallB = pygame.Rect(0, HEIGHT-wall_pixel_offset,
                                WIDTH, wall_pixel_offset)
        return [rectWallL, rectWallR, rectWallT, rectWallB]


class PlayRobot(RobotMove):
    IMG = ROBOT
    START_POS = (50, 50)
    trail_set = []
    # running game or not


run = True
images = [(BACKGROUND, (0, 0), "bg")]

wall_pixel_offset = 42

# four walls
wall_list = [Wall(30, 170, 370, 5, False),
             Wall(170, 330, 400, 5, False),
             Wall(170, 330, 5, 250, False),
             Wall(350, 500, 5, 250, False),
             Wall(0, 0, wall_pixel_offset - 1, HEIGHT, True),
             Wall(WIDTH - wall_pixel_offset, 0,
                  wall_pixel_offset, HEIGHT, True),
             Wall(0, 0, WIDTH, wall_pixel_offset - 1, True),
             Wall(0, HEIGHT - wall_pixel_offset, WIDTH, wall_pixel_offset, True)]

# the robot
player_robot = PlayRobot()

# enviroment prints
environment = Envir([800, 600])
walls = Envir.setWalls()

for wall in wall_list:
    walls.append(wall.rect)

# dt
dt = 0.1
clock = pygame.time.Clock()
FPS = 60

# display beacons on the walls
# wall_list2 =[(30, 170),(400, 170),(170, 330),(570,330),(170, 580),
#             (350, 500),(350,750),(32,746),(32,54),(568,54),(568,746)]
# for beacons in wall_list2:
#     pygame.draw.circle(SCREEN,(0, 0, 0),beacons,7)

beacons = [Beacon(30, 170, 7, SCREEN, 0), Beacon(400, 170, 7, SCREEN, 1), Beacon(170, 330, 7, SCREEN, 2), Beacon(570, 330, 7, SCREEN, 3), Beacon(170, 580, 7, SCREEN, 4),
           Beacon(350, 500, 7, SCREEN, 5), Beacon(350, 750, 7, SCREEN, 6), Beacon(32, 746, 7, SCREEN, 7), Beacon(32, 54, 7, SCREEN, 8), Beacon(568, 54, 7, SCREEN, 9), Beacon(568, 746, 7, SCREEN, 10)]


# simulation loop
while run:

    # activate quit button
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            run = False

        # timer
    clock.tick(FPS)

    # activate buttons
    keys = pygame.key.get_pressed()
    key = [keys[pygame.K_w], keys[pygame.K_s], keys[pygame.K_a],
           keys[pygame.K_d], keys[pygame.K_x]]

    # run the robot
    activate = player_robot.move(key, dt)

    # visualize objects
    environment.draw(SCREEN, images, player_robot)
    for bc in beacons:
        bc.draw(SCREEN)
    for wall in wall_list:
        wall.draw(SCREEN)
        environment.robot_frame(
            (player_robot.x, player_robot.y), player_robot.theta)

    # actual robot trajectory
    environment.trail((player_robot.x + (ROBOT.get_width()/2),
                       player_robot.y + (ROBOT.get_height()/2)))

    # estimated robot trajectory
    environment.dotted_line((player_robot.x + (ROBOT.get_width()/2),
                             player_robot.y + (ROBOT.get_height()/2)))
    # show intermediate estimates of potition of covariance
    environment.draw_elipses()
    player_robot.upd_rect()
    player_robot.draw(environment.map)

    # if (round(time.time() % 1, 1) == 0.10):
    # cast_rays(SCREEN, beacons)
    find_beacon(SCREEN, beacons)
    # ---

    pygame.display.update()

# exit the game
pygame.quit()

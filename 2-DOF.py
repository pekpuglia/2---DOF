import pygame
import Vetores
import math
import Button

pygame.init()
scrw, scrh = 800, 600
screen = pygame.display.set_mode([scrw, scrh])
timer = pygame.time.Clock()

pi = 3.141592
g = 9.81
rho = 1.225
kg = True

w, h = 10000, 5000
flight_env = pygame.Surface((w, h))


def int_vec(v):
    return [int(v[0]), int(v[1])]


class Rocket:
    def __init__(self, pad, m_f, m_prop, isp, c_d, burn_time):
        self.height = 30
        self.width = 5
        self.image = pygame.Surface((self.width, self.height))
        pygame.draw.polygon(self.image, (200, 200, 200), ([0, self.height], [self.width, self.height], [
                            self.width, self.height/4], [self.width/2, 0], [0, self.height/4]))
        self.image.set_colorkey((0, 0, 0))
        self.curr_image = self.image

        self.m_f = m_f
        self.m_prop = m_prop

        self.isp = isp
        self.c_d = c_d

        self.v = [0, 0]
        self.a = [0, 0]
        self.heading = - math.radians(pad.angle)
        self.center_pos = Vetores.soma(pad.bot_lt_pos, [self.height/2*math.cos(abs(self.heading)) - self.width/2*math.sin(abs(self.heading)),
                                                        -self.height/2*math.sin(abs(self.heading)) - self.width/2*math.cos(abs(self.heading))])
        self.burn_rate = m_prop/burn_time
        # colocar op ternario: if self.lit and self.m_prop > 0
        self.on_pad = True
        self.lit = False
        self.mthrust = 0
        self.explosion_begin = 0
        self.explosion_x = 1

    def still_on_pad(self, pad):
        """Testa se está em contato com a rampa.

        Não é um teste completo, mas basta pro lançamento."""
        if not self.on_pad:
            self.on_pad = False
            return
        # se dist(centerpos, linha longitudinal da rampa) <= ... e dist(centerpos, começo da rampa)
        m = -math.tan(math.radians(pad.angle))
        # d = |ax0 + y0 +b|/sqrt(a^2+1^2)
        # eq da reta de y - y1 = m(x - x1)
        dist_normal = abs(-m * self.center_pos[0] + self.center_pos[1] +
                          m * pad.bot_lt_pos[0] - pad.bot_lt_pos[1]) / (1 + m**2) ** 0.5
        m1 = -1/m
        dist_tang = abs(-m1 * self.center_pos[0] + self.center_pos[1] +
                        m1 * pad.bot_lt_pos[0] - pad.bot_lt_pos[1]) / (1 + m1**2) ** 0.5
        if (dist_normal <= 1.1*self.width/2) and (dist_tang <= 1.1*(pad.len + self.height/2)):
            self.on_pad = True
        else:
            self.on_pad = False

    def rocket_update(self, pad, d_t):
        self.still_on_pad(pad)

        if self.on_pad:
            self.heading = -math.radians(pad.angle)
            if self.m_prop > self.burn_rate*d_t:
                self.m_prop = self.m_prop - self.burn_rate * \
                    d_t if (self.lit) else self.m_prop
            else:
                self.m_prop = 0
        else:
            self.m_prop = self.m_prop - self.burn_rate * \
                d_t if (self.m_prop > self.burn_rate*d_t and self.lit) else 0
            self.heading = Vetores.arg(self.v)
            self.lit = (self.m_prop > 0)

    def module_update(self, pad):
        self.mw = g * (self.m_f + self.m_prop)
        self.mdrag = 0.5 * rho * \
            Vetores.mod(self.v)**2 * pi * (self.width/2)**2 * self.c_d
        self.mthrust = self.isp * g * \
            (self.burn_rate) if (self.lit and self.m_prop > 0) else 0
        self.mnormal = self.mw * \
            math.cos(math.radians(pad.angle)) if self.on_pad else 0
        self.matr = pad.mi * self.mnormal

    def newton_update(self, pad, d_t):
        """Faz todos os cálculos newtonianos necessários"""
        self.w = [0, self.mw]
        self.drag = Vetores.cis(self.mdrag, pi + Vetores.arg(self.v))
        self.thrust = Vetores.cis(self.mthrust, self.heading)
        self.normal = Vetores.cis(
            self.mnormal, -math.radians(pad.angle) - pi/2)
        self.atr = Vetores.cis(self.matr, pi - math.radians(pad.angle))

        self.R = Vetores.soma(self.thrust, self.w,
                              self.drag, self.normal, self.atr)
        if self.lit or not self.on_pad and not self.center_pos[1] > h:
            self.a = Vetores.multi(1/(self.m_f + self.m_prop), self.R)
            self.v = Vetores.soma(Vetores.multi(d_t, self.a), self.v)
            self.center_pos = Vetores.soma(
                Vetores.multi(d_t, self.v), self.center_pos)

    def write(self):
        font = pygame.font.SysFont("sourcecodeproblack", 14)
        strings = [f"Altitude: {h - self.center_pos[1]:.2f}", f"Deslocamento: {self.center_pos[0]:.2f}", f"Velocidade: {Vetores.mod(self.v):.2f}",
                   f"Massa de prop.: {self.m_prop:.2f}", f"Aceleração: {Vetores.mod(self.a):.2f}", f"Empuxo-motor: {self.mthrust:.2f}"]
        texts = [font.render(string, True, (0, 0, 0)) for string in strings]
        for i, text in enumerate(texts):
            rect = text.get_rect()
            rect.centerx = scrw - 100
            rect.centery = 50 + 40*i
            screen.blit(text, rect)

    def ground_collide(self):
        if self.center_pos[1] > h:
            # inicializa o counter
            self.explosion_begin = pygame.time.get_ticks(
            ) if self.explosion_begin == 0 else self.explosion_begin
            for y in range(self.explosion_x):
                pygame.draw.circle(
                    flight_env, (255 - 15*y, 255 - 50*y, 0), int_vec(self.center_pos), 40*self.explosion_x - 40*y)
            if pygame.time.get_ticks() - self.explosion_begin > 200*self.explosion_x:
                if pygame.time.get_ticks() - self.explosion_begin < 6 * 200:
                    self.explosion_x += 1

    def render(self):
        self.curr_image = pygame.transform.rotate(
            self.image, -(90 + math.degrees(self.heading)))
        padded_x = self.height * \
            abs(math.cos(self.heading)) + \
            self.width * abs(math.sin(self.heading))
        padded_y = self.height * \
            abs(math.sin(self.heading)) + \
            self.width * abs(math.cos(self.heading))
        blit_pos = Vetores.sub(self.center_pos, [padded_x/2, padded_y/2])
        flight_env.blit(self.curr_image, blit_pos)
        self.ground_collide()
        if self.lit:
            dist = (padded_x**2 + padded_y**2) ** 0.5 / 2
            centerVec = Vetores.cis(dist, self.heading)
            center = Vetores.sub(self.center_pos, centerVec)
            pygame.draw.circle(flight_env, (255, 0, 0),
                               int_vec(center), self.width)


class Ramp:
    def __init__(self, len, angle):
        self.len = len
        self.angle = angle  # angulo c horizontal
        self.image = pygame.Surface((self.len, 5))
        pygame.draw.rect(self.image, (50, 50, 50), [0, 0, self.len, 5])
        self.image.set_colorkey((0, 0, 0))
        self.image = pygame.transform.rotate(self.image, self.angle)
        self.bot_lt_pos = [50, h]
        self.mi = 0.03  # coef atrito

    def render(self):
        deltay = self.len * math.sin(math.radians(self.angle))
        blit_pos = Vetores.sub(self.bot_lt_pos, [0, int(deltay)])
        flight_env.blit(self.image, blit_pos)


def master_render(rocket, pad):
    # coloca bgnd no flight_env, renderiza o foguete e a rampa no flight env
    # e blita essa superficie na tela
    l = 250
    x_n = int(w/l)
    y_n = int(h/l)
    for x in range(x_n):
        for y in range(y_n):
            cor = (0, 0, 200) if (x+y) % 2 else (200, 200, 0)
            pygame.draw.rect(flight_env, cor, [x * l, y * l, l, l])
    pad.render()
    rocket.render()
    masterBlitX = 0 if rocket.center_pos[0] < scrw / \
        2 else scrw/2 - rocket.center_pos[0]
    masterBlitY = scrh - \
        h if rocket.center_pos[1] > h - scrh / \
        2 else scrh/2 - rocket.center_pos[1]
    master_blit_pos = [masterBlitX, masterBlitY]
    screen.blit(flight_env, master_blit_pos)
    rocket.write()


launch_angle = 45
rampa = Ramp(200, launch_angle)
foguete = Rocket(rampa, 100, 50, 400, 0.03, 25)
launcher = Button.Button(
    [50, 50, 100, 50], (255, 0, 0), "Launch", 20, (0, 0, 0))
time = pygame.time.get_ticks()/1000
delta_t = 0
dt = 0.001
while kg:
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            kg = False
        if ev.type == pygame.MOUSEBUTTONDOWN:
            if launcher.rect.collidepoint(pygame.mouse.get_pos()):
                foguete.lit = True
    while delta_t > dt:
        foguete.rocket_update(rampa, dt)
        foguete.module_update(rampa)
        foguete.newton_update(rampa, dt)

        delta_t -= dt
    screen.fill((0, 0, 0))
    master_render(foguete, rampa)
    launcher.update(screen)
    timer.tick(60)
    pygame.display.update()
    delta_t += pygame.time.get_ticks()/1000 - time
    time = pygame.time.get_ticks()/1000

pygame.quit()

import pygame
import Vetores
import math

pygame.init()
scrw, scrh = 800, 600
screen = pygame.display.set_mode([scrw, scrh])
timer = pygame.time.Clock()

pi = 3.141592
g = 9.81
rho = 1.225

kg = True

w, h = 3000, 5000
flight_env = pygame.Surface((w, h))


class Rocket:
    def __init__(self, pad, m_f, m_prop, isp, c_d, burn_time):
        self.height = 30
        self.width = 5
        self.image = pygame.Surface((self.width, self.height))
        pygame.draw.polygon(self.image, (0, 255, 0), ([0, self.height], [self.width, self.height], [
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
        self.mthrust = isp * g * (self.burn_rate)

        self.on_pad = True
        self.lit = True
        self.thrust = Vetores.cis(
            self.mthrust, self.heading) if self.lit else [0, 0]

    def still_on_pad(self, pad):
        """Testa se está em contato com a rampa.

        Não é um teste completo, mas basta pro lançamento."""
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
            return True
        else:
            return False

    def newton_update(self, pad, d_t):
        """Faz todos os cálculos newtonianos necessários"""
        # incluir tempo
        self.mw = g * (self.m_f + self.m_prop)
        self.w = [0, self.mw]
        self.mdrag = 0.5 * rho * \
            Vetores.mod(self.v)**2 * pi * (self.width/2)**2 * self.c_d
        self.drag = Vetores.cis(self.mdrag, pi + Vetores.arg(self.v))
        self.thrust = Vetores.cis(
            self.mthrust, self.heading) if (self.lit and self.m_prop > 0) else [0, 0]
        if self.still_on_pad(pad):
            self.mnormal = self.mw * math.cos(math.radians(pad.angle))
            self.normal = Vetores.cis(
                self.mnormal, -math.radians(pad.angle) - pi/2)
        else:
            self.mnormal = 0
            self.normal = [0, 0]
        self.matr = pad.mi * self.mnormal
        self.atr = Vetores.cis(self.matr, pi - math.radians(pad.angle))
        self.R = Vetores.soma(self.thrust, self.w,
                              self.drag, self.normal, self.atr)
        self.a = Vetores.multi(1/(self.m_f + self.m_prop), self.R)
        self.v = Vetores.soma(Vetores.multi(d_t, self.a), self.v)
        self.heading = Vetores.arg(self.v)
        self.center_pos = Vetores.soma(
            Vetores.multi(d_t, self.v), self.center_pos)
        self.m_prop -= self.burn_rate * d_t

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
        # transformar em coord inteiras
        flight_env.blit(self.curr_image, blit_pos)


class Ramp:
    def __init__(self, len, angle):
        self.len = len
        self.angle = angle  # angulo c horizontal
        self.image = pygame.Surface((self.len, 5))
        pygame.draw.rect(self.image, (255, 0, 0), [0, 0, self.len, 5])
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
    l = 100
    x_n = int(w/l)
    y_n = int(h/l)
    for x in range(x_n):
        for y in range(y_n):
            pygame.draw.rect(flight_env, (int(255/x_n * x), 0,
                                          int(255/y_n * y)), [x * l, y * l, l, l])
    pad.render()
    rocket.render()
    master_blit_pos = Vetores.sub([scrw/2, scrh/2], rocket.center_pos)
    screen.blit(flight_env, master_blit_pos)


launch_angle = 90
rampa = Ramp(200, launch_angle)
foguete = Rocket(rampa, 5, 5, 250, 0.03, 3)
time = pygame.time.get_ticks()/1000
delta_t = 0
dt = 0.01
while kg:
    for ev in pygame.event.get():
        if ev.type == pygame.QUIT:
            kg = False
    while delta_t > dt:
        foguete.newton_update(rampa, dt)
        delta_t -= dt
    screen.fill((0, 0, 0))
    master_render(foguete, rampa)
    timer.tick(60)
    pygame.display.update()
    delta_t += pygame.time.get_ticks()/1000 - time
    time = pygame.time.get_ticks()/1000

pygame.quit()

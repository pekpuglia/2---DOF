"""
Comparação de métodos de simulação de foguetes em Pygame.
Um foguete é simulado usando o método de Euler, enquanto que
o outro é simulado usando o método de Runge-Kutta de quarta ordem.
"""
import math

import pygame

import Vetores
import Button

##setando parâmetros do pygame
pygame.init()
SCRW, SCRH = 800, 600   #tamanho da tela
screen = pygame.display.set_mode([SCRW, SCRH])  #inicialização da tela
timer = pygame.time.Clock() #timer que controla o framerate
kg = True   #flag "keep going", decide quando parar a simulação


##constantes da simulação
PI = math.pi
g = 9.81
RHO = 1.225

##tela intermediária. Os foguetes são blitados no flight_env, e este é blitado posteriormente na tela
w, h = 10000, 5000
flight_env = pygame.Surface((w, h))


def int_vec(v):
    """Converte as coordenadas de v em inteiros."""
    return [int(v[0]), int(v[1])]

class Rocket:
    """Classe base para foguetes. Não deve ser instanciada por si só."""
    #é possível colocar newton update aqui. Mas sugou
    def __init__(self, pad, color, m_f, m_prop, isp, c_d, burn_time):
        #parãmetros geométricos do foguete
        self.height = 30
        self.width = 5

        #desenho da imagem poligonal do foguete
        self.image = pygame.Surface((self.width, self.height))
        pygame.draw.polygon(self.image, color, ([0, self.height], [self.width, self.height], [
                            self.width, self.height/4], [self.width/2, 0], [0, self.height/4]))
        #indica a transparência dos pixels pretos da imagem do foguete. Assim, apenas o polígono desenhado acima fica visível
        self.image.set_colorkey((0, 0, 0))
        self.curr_image = self.image
        self.color = color

        #massa seca e massa de propelente
        self.m_f = m_f
        self.m_prop = m_prop

        #impulso específico e coeficiente de arrasto do foguete
        self.isp = isp
        self.c_d = c_d

        #parâmetros cinemáticos. Velocidade, aceleração, inclinação e posição do centro.
        self.v = [0, 0]
        self.a = [0, 0]
        #note que a inclinação é medida em RADIANOS e em relação aos eixos do pygame, em sentido horário.
        self.heading = - math.radians(pad.angle)
        self.center_pos = Vetores.soma(pad.bot_lt_pos, \
            [self.height/2*math.cos(abs(self.heading)) - self.width/2*math.sin(abs(self.heading)),
             -self.height/2*math.sin(abs(self.heading)) - self.width/2*math.cos(abs(self.heading))])
        
        #taxa de consumo de combustível
        self.burn_rate = m_prop/burn_time

        #flag que indica se há contato com a rampa de lançamento
        self.on_pad = True

        #flag que indica de o motor está acionado
        self.lit = False

        #inicialização do módulo do empuxo-motor. Só é utilizado pelo método write, mas é setado por newton_update e <definir no RKROCket>
        self.mthrust = 0

        #variáveis que governam a explosão.
        self.explosion_begin = 0
        self.explosion_x = 1

    def still_on_pad(self, pos, pad):
        """Testa se está em contato com a rampa.

        Não é um teste completo, mas basta pro lançamento."""

        #se o foguete não está mais em contato com a rampa, ele nunca mais estará
        if not self.on_pad:
            return False
        
        #o teste é feito comparando a distância entre o centro do foguete e a superfície da rampa com a largura do foguete
        #e a distância entre o centro do foguete e uma reta perpendicular à superfície da rampa passando pela sua base
        #com o comprimento da rampa mais metade da altura do foguete
        m = -math.tan(math.radians(pad.angle))
        # d = |ax0 + y0 +b|/sqrt(a^2+1^2)
        # eq da reta de y - y1 = m(x - x1)
        dist_normal = abs(-m * pos[0] + pos[1] +
                          m * pad.bot_lt_pos[0] - pad.bot_lt_pos[1]) / (1 + m**2) ** 0.5
        m1 = -1/m
        dist_tang = abs(-m1 * pos[0] + pos[1] +
                        m1 * pad.bot_lt_pos[0] - pad.bot_lt_pos[1]) / (1 + m1**2) ** 0.5
        if (dist_normal <= 1.0001*self.width/2) and (dist_tang <= 1.0001*(pad.len + self.height/2)):
            return True
        else:
            return False

    def rocket_update(self, pad, d_t):
        """Atualiza parãmetros internos do foguete."""

        #Há contato com a rampa?
        self.on_pad = self.still_on_pad(self.center_pos, pad)
        
        #se há, o heading está alinhado com a rampa
        if self.on_pad:
            self.heading = -math.radians(pad.angle)
        #se não há, o foguete se alinha com a velocidade
        #fazer momentos aerodinâmicos???
        else:
            self.heading = Vetores.arg(self.v)

        #se o motor está ligado, consumir propelente
        #e checar se ainda há propelente a ser consumido
        if self.lit:
            self.m_prop = max(0, self.m_prop - self.burn_rate * d_t)
            self.lit = (self.m_prop > 0)

    def write(self, pos):
        """Escreve os parâmetros atuais do foguete na tela."""
        font = pygame.font.SysFont("sourcecodeproblack", 14)

        #lista de todos os textos a serem impressos.
        strings = [f"Altitude: {h - self.center_pos[1]:.2f}", f"Deslocamento: {self.center_pos[0]:.2f}", f"Velocidade: {Vetores.mod(self.v):.2f}",
                    f"Massa de prop.: {self.m_prop:.2f}", f"Aceleração: {Vetores.mod(self.a):.2f}", f"Empuxo-motor: {self.mthrust:.2f}"]
        
        #renderização dos textos em pygame.Surface próprias
        texts = [font.render(string, True, self.color) for string in strings]

        #para cada texto, blitar em sequência vertical
        for i, text in enumerate(texts):
            rect = text.get_rect()
            rect.centerx = SCRW - 100
            rect.centery = pos + 50 + 40*i
            screen.blit(text, rect)

    def ground_collide(self):
        """Anima uma explosão simplificada quando o foguete cai no chão."""
        #se o centro estiver abaixo do chão, seguir com a explosão
        if self.center_pos[1] > h:
            #se a explosão não tiver começado, começar
            if self.explosion_begin == 0:
                self.explosion_begin = pygame.time.get_ticks() 

            #a explosão é desenhada como uma série de círculos concêntricos com cores que são deslocadas para o círculo mais exterior
            #na primeira iteração, é desenhado um círculo amarelo
            #na segunda, o círculo amarelo é empurrado p fora por um círculo mais laranja
            #etc
            for y in range(self.explosion_x):
                pygame.draw.circle(
                    flight_env, (255 - 15*y, 255 - 50*y, 0), int_vec(self.center_pos), 40*self.explosion_x - 40*y)
            #a cada 200 ms, incrementar o número de círculos, se o número de círculos for menor que 6
            if pygame.time.get_ticks() - self.explosion_begin > 200*self.explosion_x:
                if pygame.time.get_ticks() - self.explosion_begin < 6 * 200:
                    self.explosion_x += 1

    def render(self):
        """Imprime o foguete no flight_env."""

        #rotaciona a imagem-base adequadamente
        #pygame.transform.rotate requer um ângulo em graus, em sentido anti-horário
        #a imagem base está na vertical, portanto -90 alinha a imagem com o eixo x
        #; e -math.degrees*self.heading faz a rotação final
        self.curr_image = pygame.transform.rotate(
            self.image, -(90 + math.degrees(self.heading)))
        
        #compensação dos comprimentos "acolchoados" da imagem
        padded_x = self.height * \
            abs(math.cos(self.heading)) + \
            self.width * abs(math.sin(self.heading))
        padded_y = self.height * \
            abs(math.sin(self.heading)) + \
            self.width * abs(math.cos(self.heading))
        
        #blit_pos = canto superior esquerdo da imagem
        blit_pos = int_vec(Vetores.sub(self.center_pos, [padded_x/2, padded_y/2]))
        
    
        flight_env.blit(self.curr_image, blit_pos)
    
        #testa se há colisão com o solo
        self.ground_collide()
        
        #se o motor está ligado, desenha bola de fogo
        if self.lit:
            dist = (padded_x**2 + padded_y**2) ** 0.5 / 2
            center_vec = Vetores.cis(dist, self.heading)
            center = Vetores.sub(self.center_pos, center_vec)
            pygame.draw.circle(flight_env, (255, 0, 0),
                               int_vec(center), self.width)

class EulerRocket(Rocket):
    """Classe derivada de Rocket que simula a física através do método de Euler."""

    def newton_update(self, pad, d_t):
        """Faz todos os cálculos newtonianos necessários"""

        #vetor peso
        weight = [0, g * (self.m_f + self.m_prop)]

        #vetor arrasto
        drag = Vetores.cis(0.5 * RHO * Vetores.mod(self.v)**2 * PI * (self.width/2)**2 * self.c_d, PI + Vetores.arg(self.v))
        
        #vetor empuxo
        thrust = Vetores.cis(self.isp * g * (self.burn_rate), self.heading) if (self.lit and self.m_prop > 0) else [0, 0]
        #módulo do empuxo p uso em write
        self.mthrust = Vetores.mod(thrust)
        
        #vetor força normal
        normal = Vetores.cis(Vetores.mod(weight) * math.cos(math.radians(pad.angle)), -math.radians(pad.angle) - PI/2) if self.on_pad else [0,0]
        #vetor força de atrito
        fat = Vetores.cis(pad.mi * Vetores.mod(normal), PI - math.radians(pad.angle))

        #vetor força resultante
        R = Vetores.soma(thrust, weight, drag, normal, fat)
        
        #2a lei de newton
        self.a = Vetores.multi(1/(self.m_f + self.m_prop), R)
        
        #integração por Euler
        self.v = Vetores.soma(Vetores.multi(d_t, self.a), self.v)
        self.center_pos = Vetores.soma(Vetores.multi(d_t, self.v), self.center_pos)

    def update(self, pad, d_t):
        self.rocket_update(pad, d_t)

        #condições de atualização:
        #o foguete deve estar acima do solo;
        #se o foguete estiver na rampa e desligado, não atualizar (senão ele escorregaria p baixo)
        if self.lit or not self.on_pad and not self.center_pos[1] > h:
            self.newton_update(pad, d_t)



class RKRocket(Rocket):
    """Classe derivada de Rocket que simula a física através do método de Runge - Kutta de quarta ordem."""
    def f(self, t, state_vec, pad):
        """dY/dt = f. É a expressão das derivadas de todos os parâmetros relevantes.
        
        state_vec =  [x, y, vx, vy]
        heading, drag, thrust, normal, fat todas calculadas a partir disso
        t = (1 ou 1/2) * dt
        """

        #f e EulerRocket.newton_update repetem uma parte do código, mas sugou unificar.
        #um dia quem sabe...

        #cria variáveis locais p uso
        x, y, vx, vy = state_vec

        #alinha o foguete com a velocidade ou com a rampa. Equivalente a 191-196. Fazer com momentos aerodinâmicos?
        heading = Vetores.arg(Vetores.soma([vx, 0], [0, vy])) if not self.on_pad else -math.radians(pad.angle)

        #consome combústivel
        m_prop = max(self.m_prop - t*self.burn_rate, 0) if self.lit else self.m_prop
        m_t = self.m_f + m_prop

        #vetor peso
        weight = [0, m_t * g]

        #vetor arrasto
        d = Vetores.cis(0.5 * RHO * (vx**2 + vy**2) * PI * (self.width/2)**2 * self.c_d, PI + heading)

        #vetor empuxo
        thrust = Vetores.cis(self.isp * g * (self.burn_rate), heading) if (self.lit and m_prop > 0) else [0, 0]
        self.mthrust = Vetores.mod(thrust)      #@TODO mudar de lugar? Aqui, no fim do frame ele mostra o empuxo de k3

        #vetor força normal
        n = Vetores.cis(m_t * g * math.cos(math.radians(pad.angle)), -math.radians(pad.angle) - PI/2) if self.still_on_pad([x, y], pad) else [0, 0]
    
        #vetor força de atrito
        fat = Vetores.cis(pad.mi * Vetores.mod(n), PI - math.radians(pad.angle))

        #vetor força resultante
        r = Vetores.soma(weight, d, thrust, n, fat)

        #retorna as derivadas do vetor de estado
        return [vx, vy, r[0]/m_t, r[1]/m_t]

    def rk_step(self, d, pad):
        """Implementação do método de Runge - Kutta.
        Retorna a variação do vetor de estado no intervalo de tempo d dado."""
        
        #nomes locais p funções
        s = Vetores.soma
        m = Vetores.multi
        
        #vetor de estado inicial
        y = [self.center_pos[0], self.center_pos[1], self.v[0], self.v[1]]
        
        #implementação do rk4
        k0 = self.f(0  , y           , pad)
        k1 = self.f(d/2, s(y, m(d/2, k0)), pad) 
        k2 = self.f(d/2, s(y, m(d/2, k1)), pad)
        k3 = self.f(d  , s(y, m(d  , k2)), pad)
        delta_y = m(d/6, s(k0, m(2, k1), m(2, k2), k3) )
        
        #cálculo da aceleração, p uso no método write
        self.a = m(1/d, delta_y[2:])
        
        #retorna a variação no vetor de estado
        return delta_y

    def update(self, pad, d_t):
        """Faz todos os cálculos newtonianos necessários"""
        self.rocket_update(pad, d_t)

        #condições de atualização:
        #o foguete deve estar acima do solo;
        #se o foguete estiver na rampa e desligado, não atualizar (senão ele escorregaria p baixo)
        if self.lit or not self.on_pad and not self.center_pos[1] > h:

            #junta a variação de rk_step com o vetor de estado atual
            state = Vetores.soma(self.rk_step(d_t, pad), [*self.center_pos, *self.v])
            self.center_pos, self.v = state[:2], state[2:]

class Ramp:
    """Representa a plataforma de lançamento.
    Pode ser inclinada a qualquer ângulo."""
    def __init__(self, length, angle):

        #comprimento da rampa
        self.len = length

        # angulo c horizontal
        self.angle = angle

        #cria a imagem e a rotaciona adequadamente
        self.image = pygame.Surface((self.len, 5))
        pygame.draw.rect(self.image, (50, 50, 50), [0, 0, self.len, 5])
        self.image.set_colorkey((0, 0, 0))
        self.image = pygame.transform.rotate(self.image, self.angle)
        
        #fixa a parte inferior esquerda da rampa
        self.bot_lt_pos = [50, h]
        
        # coef atrito. Para uso nos métodos físicos dos foguetes
        self.mi = 0.03  

    def render(self):
        """Imprime a imagem da rampa no flight_env."""

        #encontra o canto superior esquerdo e blita
        deltay = self.len * math.sin(math.radians(self.angle))
        blit_pos = Vetores.sub(self.bot_lt_pos, [0, int(deltay)])
        flight_env.blit(self.image, blit_pos)


def master_render(rockets, pad):
    """Renderiza rampa e foguete[s] no flight_env e imprime este na tela."""

    #desenha um padrão alternante de quadrados azuis e amarelos
    #para performance, seria melhor ter um flight_env base com esses quadrados desenhados,
    #mas a simulação roda bem mesmo assim
    l = 250
    x_n = int(w/l)
    y_n = int(h/l)
    for x in range(x_n):
        for y in range(y_n):
            cor = (0, 0, 200) if (x+y) % 2 else (150, 150, 0)
            pygame.draw.rect(flight_env, cor, [x * l, y * l, l, l])
    
    #renderiza cada elemento
    pad.render()
    rockets[0].render()
    rockets[1].render()

    #escolhe qual foguete seguir. Fazer botão p alternar?
    rocket = rockets[0]

    #coloca o foguete escolhido no centro da tela, desde que ele não esteja nas margens de flight_env
    masterBlitX = 0 if rocket.center_pos[0] < SCRW / 2 else SCRW / 2 - rocket.center_pos[0]
    masterBlitY = SCRH - h if rocket.center_pos[1] > h - SCRH / 2 else SCRH / 2 - rocket.center_pos[1]
    master_blit_pos = [masterBlitX, masterBlitY]
    screen.blit(flight_env, master_blit_pos)
    
    #escreve os parâmetros de cada foguete na tela em posições distintas
    rockets[0].write(0)
    rockets[1].write(SCRH/2)


#inicialização da rampa
LAUNCH_ANGLE = 75
rampa = Ramp(200, LAUNCH_ANGLE)

#inicialização dos foguetes
#os parâmetros dos foguetes são unificados para fins de comparação
params = [50, 50, 400, 0.03, 5]
fogueteuler = EulerRocket(rampa, (255, 0, 0), *params)
fogueterk = RKRocket(rampa, (0, 255, 0), *params)

#botão de lançamento
launcher = Button.Button(
    [50, 50, 100, 50], (255, 0, 0), "Launch", 20, (0, 0, 0))

#tempo de início da simulação
time = pygame.time.get_ticks()/1000

#tempo transcorrido desde o último frame
delta_t = 0

#intervalo de tempo a ser utilizado pela simulação da física
dt = 0.01

#loop da simulação
while kg:
    for ev in pygame.event.get():
        
        #sai do pygame
        if ev.type == pygame.QUIT:
            kg = False
        
        #testa se houve clique no botão de lançamento
        if ev.type == pygame.MOUSEBUTTONDOWN:
            if launcher.rect.collidepoint(pygame.mouse.get_pos()):
                #ativa os motores
                fogueteuler.lit = True
                fogueterk.lit = True

    #simulação da física

    #o intervalo entre dois frames é divido em vários intervalos de tamanho constante e pequeno
    #o tempo que "sobra" é guardado pro próximo frame p manter a precisão
    while delta_t > dt:
        fogueteuler.update(rampa, dt)
        fogueterk.update(rampa, dt)
        delta_t -= dt

    #apaga o frame anterior
    screen.fill((0, 0, 0))

    #renderiza todos os updates
    master_render([fogueteuler, fogueterk], rampa)
    
    #sobrepõe o botão ao flight_env
    launcher.update(screen)

    #limita o framerate a 60FPS
    timer.tick(60)

    #atualiza o display
    pygame.display.update()

    #calcula o tempo gasto desde o último frame
    delta_t += pygame.time.get_ticks()/1000 - time
    
    #tempo do fim deste frame p ser usado no próximo
    time = pygame.time.get_ticks()/1000

pygame.quit()

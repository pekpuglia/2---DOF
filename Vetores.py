import math


def soma(*v):
    """Soma vetores."""
    lens = [len(vec) for vec in v]
    for leng in lens:
        if leng != max(lens):
            raise TypeError("Different lengths!")
    result = [0 for x in range(lens[0])]
    for vec in v:
        for i, el in enumerate(vec):
            result[i] += el
    return result


def sub(v1, v2):
    """Subtrai o segundo do primeiro."""
    return [v1[0] - v2[0], v1[1] - v2[1]]


def mod(v):
    """Calcula o módulo."""
    return (v[0]**2 + v[1]**2)**0.5


def esc(v1, v2):
    """Calcula o produto escalar."""
    return v1[0]*v2[0]+v1[1]*v2[1]


def ang(v1, v2):
    return math.degrees(math.acos(esc(v1, v2) / (mod(v1)*mod(v2))))


def vet(v1, v2):
    return v1[0]*v2[1] - v1[1]*v2[0]


def dist(v1, v2):
    """Calcula a distância."""
    return (sub(v1, v2)[0]**2 + sub(v1, v2)[1]**2)**(1/2)


def multi(k, v):
    """Multiplica um vetor por um escalar."""
    return [k * el for el in v]


def versor(v):
    """Retorna o versor paralelo ao vetor dado."""
    return multi(1/mod(v), v)


def perpendicular(v):
    """Cria um versor rotacionado pi/2 em sentido anti-horário"""
    # exceto no pygame
    # na verdade, aumenta o argumento em pi/2
    return [-versor(v)[1], versor(v)[0]]


def rot(v, theta):
    """Rotaciona v de theta radianos"""
    return [math.cos(theta)*v[0]-math.sin(theta)*v[1],
            math.cos(theta)*v[1]+math.sin(theta)*v[0]]


def arg(v):
    """Retorna o argumento de v, em radianos"""
    try:
        if v[1] >= 0:
            return math.acos(v[0] / (v[0]**2+v[1]**2)**0.5)
        else:
            return 2*math.pi - math.acos(v[0] / (v[0]**2+v[1]**2)**0.5)
    except ZeroDivisionError:
        return 0


def cis(mod, arg):
    """Converte forma polar em cartesiana, arg em rad"""
    return [mod*math.cos(arg), mod*math.sin(arg)]

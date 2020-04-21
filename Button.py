import pygame
pygame.init()


class Button:
    """Botões para os menus;"""

    def __init__(self, rect, bgcolor, string, size, txtcolor):
        self.rect = pygame.Rect(rect)
        self.bgcolor = bgcolor
        self.string = string
        # tamanho do texto
        self.size = size
        self.font = pygame.font.SysFont("sourcecodeproblack", self.size)
        self.text = self.font.render(string, True, txtcolor)
        self.text_rect = self.text.get_rect()
        self.text_rect.centerx = self.rect.width/2
        self.text_rect.centery = self.rect.height/2
        self.button = pygame.Surface((self.rect.width, self.rect.height))
        self.button.fill(bgcolor)
        self.button.blit(self.text, self.text_rect)

    def update(self, screen):
        screen.blit(self.button, self.rect)

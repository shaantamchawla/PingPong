import pygame
import pygooey
import serial
import time

class FrontPanel:
	def __init__(self):
		self.serial_port = '/dev/tty.usbmodem14601'
		self.ser = serial.Serial(self.serial_port, baudrate=115200, 
			parity=serial.PARITY_NONE,
			stopbits=serial.STOPBITS_ONE,
			bytesize=serial.EIGHTBITS)

		self.message_length = 6
		self.data_in = None
		self.x, self.y, self.r, self.th, self.ph, self.dt = None,None,None,None,None,None

	def textbox_callback(self, id, final):
		print('enter pressed, textbox contains {}'.format(final))

	def alternative_callback(self, id, final):
		print('alternative textbox contains {}'.format(final))

	def button_callback(self):
		str_to_write = str(self.entry.final) + ',' + str(self.entry2.final) + '\n'
		print('button pressed, textbox contains {}'.format(self.entry.final))
		print(str_to_write)

		self.write_commands(bytes(str_to_write.encode("utf-8")))
  
	## our functions

	def write_commands(self, str_to_write):
		self.ser.write(str_to_write)

	def print_on_enter(self, id, final):
		print('enter pressed, textbox contains {}'.format(final))

	def render_circle(self):
		import random
        #for grid system
		if self.x and self.y and self.r:
			self.myNewSurface.fill((255, 255, 127))
			pygame.draw.circle(self.myNewSurface, (0,0,0), [0 + random.randint(0, self.x), 0 + random.randint(0, self.y)], self.r, 0)
			#blit myNewSurface onto the main screen at the position (0, 0)
			self.screen.blit(self.myNewSurface, (200, 500))

	def main(self):
		background_colour = (255, 255, 127)
		widgets = []
		self.screen = pygame.display.set_mode((800, 800))
		self.screen_rect = self.screen.get_rect()
		pygame.display.set_caption('Front Panel — Ping Pong — Team MicroProfessors')
		self.screen.fill(background_colour)
		pygame.display.flip()

		my_font = pygame.font.SysFont('GAMES_FONT', 30)
		
		text_surface = my_font.render('Motor X control', False, (0, 0, 0))
		self.screen.blit(text_surface, (175,260))

		text_surface2 = my_font.render('Motor Y control', False, (0, 0, 0))
		self.screen.blit(text_surface2, (175,360))

		text_surface3 = my_font.render('Circle Path', False, (0, 0, 0))
		self.screen.blit(text_surface3, (175,460))

		self.entry_settings = {
		"inactive_on_enter" : False,
		'active':False
		}
		
		self.entry = pygooey.TextBox(rect=(350,230,150,60), command=self.textbox_callback, **self.entry_settings)
		widgets.append(self.entry)
		self.entry2 = pygooey.TextBox(rect=(350,330,150,60), command=self.alternative_callback, **self.entry_settings)
		widgets.append(self.entry2)

		self.myNewSurface = pygame.Surface((400, 300))

		#change its background color
		self.myNewSurface.fill((55,155,255))

		#see all settings help(pygooey.Button.__init__)
		btn_settings = {
		    "clicked_font_color" : (0,0,0),
		    "hover_font_color"   : (205,195, 100),
		    'font'               : pygame.font.Font(None,16),
		    'font_color'         : (255,255,255),
		    'border_color'       : (0,0,0),
		}

		btn = pygooey.Button(rect=(300,30,205,55), command=self.button_callback, text='Send Motor Commands', **btn_settings)
		widgets.append(btn)

		# game loop

		clock = pygame.time.Clock()

		counter = 0
		# Variable to keep our game loop running
		running = True

		while running:
			self.data_in = self.ser.readline().decode("utf-8").split(",")

			if len(self.data_in) == self.message_length: ## Validate packet
				self.x, self.y, self.r, self.th, self.ph, self.dt = [float(val) for val in self.data_in]
				print(self.x, self.y, self.r, self.th, self.ph, self.dt)

			# for loop through the event queue  
			for event in pygame.event.get():
				# Check for QUIT event      
				if event.type == pygame.QUIT:
					running = False

				for w in widgets:
					w.get_event(event)

			for w in widgets:
				w.update()
				w.draw(self.screen)

			counter += 1

			self.render_circle()
			pygame.display.update()

		pygame.quit()

f = FrontPanel()
f.main()
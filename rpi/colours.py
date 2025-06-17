import numpy as np

class Blue():
    h_min = 63
    s_min = 61
    v_min = 102
    h_max = 152
    s_max = 216
    v_max = 255

    # [91, 94, 0], [136, 255, 255]

class Yellow():
    h_min = 25
    s_min = 37
    v_min = 160
    h_max = 35
    s_max = 160
    v_max = 255

class Green():
    h_min = 40
    s_min = 46
    v_min = 0
    h_max = 50
    s_max = 255
    v_max = 255

class Obstacle():

    h_min = 142
    s_min = 105
    v_min = 103
    h_max = 179
    s_max = 160
    v_max = 179

class Arrow():
    h_min = 30
    s_min = 100
    v_min = 172
    h_max = 147
    s_max = 255
    v_max = 255

class MarkerBlue():
    h_min = 84
    s_min = 46
    v_min = 108
    h_max = 98
    s_max = 255
    v_max = 255

class MarkerOrange():
    h_min = 175
    s_min = 56
    v_min = 156
    h_max = 179
    s_max = 255
    v_max = 255

class MarkerGreen():
    h_min = 71
    s_min = 71
    v_min = 110
    h_max = 82
    s_max = 255
    v_max = 255

class TapeBlue():
    lower= np.array([93, 56, 0] )
    upper = np.array([115, 255, 255])

class TapeYellow():
    lower = np.array([7, 53, 0])
    upper = np.array([36, 255, 255])

class HighlighterPink():
    lower = np.array([146, 55, 0]) 
    upper = np.array([179, 255, 255])

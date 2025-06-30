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
    h_min = 96
    s_min = 40
    v_min = 164
    h_max = 124
    s_max = 255
    v_max = 247

class TapeYellow():
    h_min = 14
    s_min = 47
    v_min = 103
    h_max = 36
    s_max = 191
    v_max = 230

class PaintPurple():
    h_min = 148
    s_min = 71
    v_min = 112
    h_max = 179
    s_max = 182
    v_max = 153

class TapeGreen():
    h_min = 39
    s_min = 28
    v_min = 125
    h_max = 73
    s_max = 77
    v_max = 182

class HighlighterPink():
    lower = np.array([146, 55, 0]) 
    upper = np.array([179, 255, 255])

class TapeGrey():
    lower = np.array([82, 0, 146])
    upper = np.array([106, 47, 255])
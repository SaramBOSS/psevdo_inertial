
import numpy as np
import scipy as sc
import pandas as pd

#система срабатывает по событию изменения датчиков

#v_xm0, v_ym0 -- скорости в мировой плоскости
#a_x, a_y, a_z -- данные акселерометра
#g_x, g_y, g_z -- текущие g-компоненты


#phi_x0, phi_y0, phi_z0 -- углы поворота
#w_x, w_y, w_z -- данные гироскопа

def calc_world_direction_b(a_x, a_y, h_x, h_y, b_xy):
    b_xy = np.abs(b_xy)
    h2 = np.abs(h_y)
    h1 = np.abs(h_x)
    b_xy_m = 0
    if a_x * a_y >= 0:
        # случай трапеции
        dh = np.abs(h2 - h1)
        b_xy_m = np.sqrt(b_xy ** 2 - dh ** 2)
    else:
        # случай двух треугольников
        h2 = np.abs(h_y)
        h1 = np.abs(h_x)
        b1 = b_xy / (h2 / h1 + 1)
        b2 = b_xy / (h1 / h2 + 1)
        b_xy_m1 = np.sqrt(b1 ** 2 - h1 ** 2)
        b_xy_m2 = np.sqrt(b2 ** 2 - h2 ** 2)
        b_xy_m = b_xy_m1 + b_xy_m2

    return b_xy_m

def initialize(a_x, a_y, a_z, w_x, w_y, w_z): #пытаемся инициализироваться (в состоянии покоя)
    g_thresh = 0.03
    w_thresh = 0.03
    g = sc.constants.g
    errno = 1
    # пусть g_x, g_y, g_z -- g-компоненты
    if (np.abs(np.sqrt(a_x ** 2 + a_y ** 2 + a_z ** 2) - g) < g_thresh) and ((w_x**2 + w_y**2 + w_z**2) > w_thresh**2):
        # ищем g-компоненты, углы отклонения от мировой плоскости m
        # g - компоненты
        g_x = a_x
        g_y = a_y
        g_z = a_z

        # углы отклонения от g
        alpha_x = np.arccos(g_x / g)
        alpha_y = np.arccos(g_y / g)
        alpha_z = np.arccos(g_z / g)

        return errno, g_x, g_y, g_z, alpha_x, alpha_y, alpha_z
    else:
        errno = 0
        return errno, a_x, a_y, a_z, 0, 0, 0

def calcMedianOfAbsolutes(df_acc0):
    cols_acc = df_acc0.columns()
    accs0 = np.sqrt(df_acc0[cols_acc[1]][:] ** 2 + df_acc0[cols_acc[2]][:] ** 2 + df_acc0[cols_acc][3][:] ** 2)
    acc0 = np.median(accs0)
    return acc0

def isMapMovement(df_gyro, time_win=100, w_thresh=2.5):
    #cols [timestamps, acc_x, acc_y, acc_z]
    #cols [timestamps, gyro_x, gyro_y, gyro_z]

    # time_acc0 = df_acc['timestamps'][0]
    # time_acc1 = df_acc['timestamps'][1]

    time_gyro0 = df_gyro['timestamps'][0]
    time_gyro1 = df_gyro['timestamps'][1]

    I = -1
    # if time_acc1 - time_acc0 < 0:
    #     return I
    if time_gyro1 - time_gyro0 < time_win:
        return I

    # cols_acc = df_acc.columns()
    # accs = np.sqrt(df_acc[cols_acc[1]][:]**2 + df_acc[cols_acc[2]][:]**2 + df_acc[cols_acc][3][:]**2)

    # cols_gyro = df_gyro.columns()
    # gyros = np.sqrt(df_gyro[cols_gyro[1]][:] ** 2 + df_gyro[cols_gyro[2]][:] ** 2 + df_gyro[cols_gyro[3]][:] ** 2)
    # med_gyr = np.median(gyros)
    med_gyr = calcMedianOfAbsolutes(df_gyro)

    if med_gyr > w_thresh:
        I = 1
    else:
        I = 0

    return I

#def isFall(df_acc0, df_gyro0, df_acc1, df_gyro1, time_win=100, a_thresh=5, w_thresh=5):
def isFall(df_gyro0, df_gyro1, time_win=100, w_thresh=5):

    # cols_acc = df_acc0.columns()
    # accs0 = np.sqrt(df_acc0[cols_acc[1]][:]**2 + df_acc0[cols_acc[2]][:]**2 + df_acc0[cols_acc][3][:]**2)
    # acc0 = np.median(accs0)
    # acc0 = calcMedianOfAbsolutes(df_acc0)
    # acc1 = calcMedianOfAbsolutes(df_acc1)

    gyro0 = calcMedianOfAbsolutes(df_gyro0)
    gyro1 = calcMedianOfAbsolutes(df_gyro1)

    I = 0
    if (np.abs(gyro1 - gyro0) > w_thresh): # and (np.abs(acc1 - acc0) > a_thresh):
        I = 1
    return I

def isHit(df_gyro1, df_gyro2, df_acc0, df_acc1, time_win=100, a_thresh=5, w_thresh_down=2):
    acc0 = calcMedianOfAbsolutes(df_acc0)
    acc1 = calcMedianOfAbsolutes(df_acc1)

    gyro1 = calcMedianOfAbsolutes(df_gyro1)
    gyro2 = calcMedianOfAbsolutes(df_gyro2)

    I = 0
    if  (np.abs(acc1 - acc0) > a_thresh):
        #удар
        I = 1
        if (np.abs(gyro2 - gyro1) > w_thresh_down):
            #резкий старт
            I = 0

    return I

def is_hit(csv_string_gyr1, csv_string_gyr2, csv_string_acc0, csv_string_acc1):
    columns = ['timestamps', 'gyro_x', 'gyro_y', 'gyro_z']
    df_gyr1 = pd.DataFrame([row.split(',') for row in csv_string_gyr1.split('\n')],
                       columns=columns)

    df_gyr2 = pd.DataFrame([row.split(',') for row in csv_string_gyr2.split('\n')],
                       columns=columns)

    columns = ['timestamps', 'acc_x', 'acc_y', 'acc_z']

    df_acc0 = pd.DataFrame([row.split(',') for row in csv_string_acc0.split('\n')],
                           columns=columns)

    df_acc1 = pd.DataFrame([row.split(',') for row in csv_string_acc1.split('\n')],
                           columns=columns)

    I = isHit(df_gyr1, df_gyr2, df_acc0, df_acc1, time_win=100, a_thresh=5, w_thresh_down=2)

def is_fall(csv_string0, csv_string1):
    time_win_len = 1000
    movement_thresh = 1
    #df = pd.read_csv('./test_acc1.csv', index_col=0)
    columns = ['timestamps', 'gyro_x', 'gyro_y', 'gyro_z']
    df0 = pd.DataFrame([row.split(',') for row in csv_string0.split('\n')],
                        columns=columns)

    df1= pd.DataFrame([row.split(',') for row in csv_string0.split('\n')],
                       columns=columns)

    #cols = df.columns
    I = isFall(df0, df1, time_win=100, w_thresh=2.5)
    return str(I)

def is_movement(csv_string):
    time_win_len = 1000
    movement_thresh = 1
    #df = pd.read_csv('./test_acc1.csv', index_col=0)
    columns = ['timestamps', 'acc_x', 'acc_y', 'acc_z']
    df = pd.DataFrame([row.split(',') for row in csv_string.split('\n')],
                        columns=columns)
    #cols = df.columns
    I = isMapMovement(df, time_win=100, w_thresh=2.5)
    return str(I)

if __name__ == '__main__':
    #df = pd.read_csv('./logs_1.txt')
    csv_string = "1, 2, 3, 4\n5, 6, 7, 8"
    I = is_movement(csv_string)
    print(I)




# def acc_changed(a_x, a_y, a_z, a_x_prev, a_y_prev, a_z_prev, g_x, g_y, g_z, alpha_x, alpha_y, alpha_z,
#                 v_xmc_prev, v_ymc_prev, t_acc_prev, t_acc):
#     g_thresh = 0.03
#     #
#     MODE = 'stay'
#     g = sc.constants.g
#     #пусть g_x, g_y, g_z -- g-компоненты
#     if np.abs(np.sqrt(a_x**2 + a_y**2 + a_z**2) - g) < g_thresh:
#         #ищем g-компоненты, углы отклонения от мировой плоскости m
#         #g - компоненты
#         g_x = (a_x + a_x_prev)/2
#         g_y = (a_y + a_y_prev)/2
#         g_z = (a_z + a_z_prev)/2
#
#         #углы отклонения от мировой плоскости m
#         alpha_x = np.arcsin(g_x/g)
#         alpha_y = np.arcsin(g_y/g)
#         alpha_z = np.arcsin(g_z/g)
#
#         v_ymc = v_ymc_prev
#         v_xmc = v_xmc_prev
#         return MODE, g_x, g_y, g_z, alpha_x, alpha_y, alpha_z, v_xmc, v_ymc
#     else:
#         MODE = 'move'
#
#         # вычитаем g-компоненты
#         a_x = a_x - g_x
#         a_y = a_y - g_y
#         a_z = a_z - g_z
#
#         a_x_prev = a_x_prev - g_x
#         a_y_prev = a_y_prev - g_y
#         a_z_prev = a_z_prev - g_z
#
#         #вычисляем среднее ускорение в данные тик
#         a_x = (a_x + a_x_prev)/2
#         a_y = (a_y + a_y_prev)/2
#         a_z = (a_z + a_z_prev)/2
#
#         #ищем проекции на мировую плоскость
#         a_xm = a_x*np.cos(alpha_x)
#         a_ym = a_y*np.cos(alpha_y)
#         a_zm = a_z*np.cos(alpha_z)
#
#         #ищем углы между проекциями #вычисление направлений на мировой плоскости
#
#         h_x = a_x*np.sin(alpha_x)
#         h_y = a_y*np.sin(alpha_y)
#         h_z = a_z*np.sin(alpha_z)
#
#         b_xy = np.sqrt(a_x**2 + a_y**2)
#         b_yz = np.sqrt(a_y**2 + a_z**2)
#         #b_zx = np.sqrt(a_z**2 + a_z**2)
#
#         b_xy_m = calc_world_direction_b(a_x, a_y, h_x, h_y, b_xy)
#         b_yz_m = calc_world_direction_b(a_y, a_z, h_y, h_z, b_yz)
#         alpha_xy = np.arccos((a_xm**2 + a_ym**2 - b_xy_m**2)/(2*np.abs(a_xm)*np.abs(a_ym)))
#         alpha_yz = np.arccos((a_ym**2 + a_zm**2 - b_yz_m**2))/(2*np.abs(a_ym)*np.abs(a_zm))
#
#         #ось y_m -- мировой плоскости направляем по начальной проекции оси oy
#         #ось x_m -- перпендикулярна y_m, тогда
#
#         a_xm_y_m = a_xm*np.cos(alpha_xy)
#         a_xm_x_m = a_xm*np.sin(alpha_xy)
#
#         a_ym_y_m = a_ym
#         a_ym_x_m = 0
#
#         a_zm_y_m = a_zm*np.cos(alpha_yz)
#         a_zm_x_m = a_zm*np.sin(alpha_yz)
#
#         #компоненты ускорения на мировой плоскости
#         a_ymc = a_xm_y_m + a_ym_y_m + a_zm_y_m
#         a_xmc = a_xm_x_m + a_ym_x_m + a_zm_x_m
#
#         #компоненты скорости на мировой плоскости
#         v_ymc = a_ymc*(t_acc - t_acc_prev)
#         v_xmc = a_xmc*(t_acc - t_acc_prev)
#
#     # (a_x, a_y, a_z, a_x_prev, a_y_prev, a_z_prev, g_x, g_y, g_z, alpha_x, alpha_y, alpha_z,
#     #  v_xmc_prev, v_ymc_prev, t_acc_prev, t_acc):
#
#     return MODE, g_x, g_y, g_z, alpha_x, alpha_y, alpha_z, v_xmc, v_ymc
#
# def gyro_changed(w_x, w_y, w_z, w_x_prev, w_y_prev, w_z_prev, g_x, g_y, g_z, alpha_x, alpha_y, alpha_z,
#                  v_xmc_prev, v_ymc_prev, t_gyro_prev, t_gyro):
#     # смена систем координат и изменение компонент
#     return 0

#def inertial_system(a_x, a_y, a_z, w_x, w_y, w_z, )
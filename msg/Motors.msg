#-------------------------------------------------------------------------#
# File: Motors.msg                                                        #
# Autor: FIE Equipo de robótica autónoma                                  #
# Developers:                                                             #
#   Maceira Coni, Carlos Alberto <cmaceira@fie.undef.edu.ar>              #
#   Muena, Guillermo Ariel       <gmuena@fie.undef.edu.ar>                #
# Brief:                                                                  #
#   Mensaje personalizado de ROS para la tracción de motores. 'data_l' es #
#   un float que indica la fracción entre 0 y 1 correspondiente a la ve-  #
#   locidad porcentual (0% a 100% respectivamente) que corresponde a los  #
#   dos motores de tracción del lateral izquierdo del robot. 'data_r' es  #
#   lo mismo que data_l pero para los motores de tracción derecha.        #
#-------------------------------------------------------------------------#
# Changelog:                                                              #
# 2021-12-06: Commentaries, Header and Changelog Added                    #
#-------------------------------------------------------------------------#
float32 data_l
float32 data_r

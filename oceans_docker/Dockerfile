FROM docker.cogrob.com/ros_kinetic_base

MAINTAINER Michelle Sit

RUN apt-get -y update

COPY install_gazebo.sh install_general_env.sh install_uuv_pkgs.sh /root/install/
RUN chmod +x /root/install/install_gazebo.sh
RUN chmod +x /root/install/install_general_env.sh
# RUN chmod +x /root/install/install_uuv_pkgs.sh

RUN /root/install/install_general_env.sh
RUN /root/install/install_gazebo.sh
# RUN /root/install/install_uuv_pkgs.sh

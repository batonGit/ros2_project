import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'gps_navigation' # Убедитесь, что это имя соответствует имени папки пакета и package.xml

setup(
    name=package_name,
    version='0.0.0', # Обновите версию при необходимости
    packages=find_packages(exclude=['test']), # Автоматически найдет вложенную папку gps_navigation
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '* launch.[pxy][yma]*'))),
        # Добавьте сюда установку вашего конфиг файла
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ваш_имя', # Укажите ваше имя
    maintainer_email='ваш_email@example.com', # Укажите ваш email
    description='Navigation package including GPS navigation and manager node', # Описание пакета
    license='Apache-2.0', # Лицензия
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'navigation_manager_node = gps_navigation.navigation_manager_node:main',
            # Если у вас будут другие Python узлы в этой же папке, добавьте их здесь
        ],
    },
)
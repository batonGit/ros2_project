import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robot_navigation_manager' # Имя вашего НОВОГО пакета

setup(
    name=package_name,
    version='0.0.0', # Обновите версию при необходимости
    packages=find_packages(exclude=['test']), # Должен найти папку robot_navigation_manager внутри src/robot_navigation_manager
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*.[pxy][yma]*'))), # Если у вас будут launch файлы
        # ДОБАВЛЕНА СТРОКА для установки папки config
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root', # Укажите актуальные данные
    maintainer_email='root@todo.todo', # Укажите актуальные данные
    description='Robot navigation manager node', # Обновите описание
    license='Apache-2.0', # Укажите актуальную лицензию
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # УКАЗАНО ИМЯ НОВОГО ПАКЕТА И УЗЛА
            'navigation_manager_node = robot_navigation_manager.navigation_manager_node:main',
        ],
    },
)
# Last updated: 2026-07-27
from setuptools import find_packages, setup

package_name = 'pub_test'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        # numpy/scikit-learn/opencv는 package.xml의 exec_depend(rosdep, apt)가 담당하므로 여기서 뺌.
        # 아래는 rosdep 데이터베이스에 없는 pip 전용 라이브러리만 — Signal-Vision의 pyproject.toml과
        # 버전 맞출 것(drift 주의). colcon의 ament_python 빌드는 pip을 --no-deps로 호출하므로
        # install_requires는 colcon build로 설치되지 않는다(버전 고정용 문서일 뿐).
        # 노드 실행은 scripts/setup_infer_env.py로 만든 venv(.venv-infer)에서 해야 한다.
        'mediapipe==0.10.35',
        'torch==2.12.1',
        'websocket-client==1.9.0',
        'tqdm==4.68.4',
    ],
    zip_safe=True,
    maintainer='sho',
    maintainer_email='shp9826@naver.com',
    description='Publish Topic Test',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pub_node=pub_test.topic_pub_base:main'
        ],
    },
)

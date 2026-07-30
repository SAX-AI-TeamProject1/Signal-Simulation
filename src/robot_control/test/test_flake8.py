# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from pathlib import Path

from ament_flake8.main import main_with_errors
import pytest

# robot_control/vision_hand 는 Signal-Vision 을 그대로 벤더링한 사본이다
# (scripts/setup_infer_env.py 가 갱신한다). 스타일을 여기서 고쳐 봐야 다음 갱신 때
# 통째로 덮어써지고, 원본과 diff 도 어긋난다. 그래서 검사 대상에서 뺀다.
# 우리가 쓴 코드(camera_node/, metrics.py, launch/, setup.py)는 그대로 검사한다.
VENDORED = str(Path(__file__).resolve().parent.parent / 'robot_control' / 'vision_hand')


@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=['--exclude', VENDORED])
    assert rc == 0, \
        'Found %d code style errors / warnings:\n' % len(errors) + \
        '\n'.join(errors)

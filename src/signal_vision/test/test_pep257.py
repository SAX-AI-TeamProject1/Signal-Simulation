# Copyright 2015 Open Source Robotics Foundation, Inc.
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

from ament_pep257.main import main
import pytest


@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    # vision_hand/ 는 Signal-Vision 에서 통째로 복사해 오는 벤더 사본이라 이 리포지토리의
    # 스타일 규칙을 강제할 대상이 아니다 — 고쳐 봐야 다음 태스크 4 실행 때 덮인다.
    # flake8 쪽은 vision_hand/AMENT_IGNORE 로 걸러지지만(ament_flake8 이 그 마커를 본다),
    # ament_pep257 은 AMENT_IGNORE 를 안 보고 --exclude 만 본다. 그래서 여기서만 명시한다.
    # (--exclude 는 abspath commonpath 비교라 디렉터리로 동작한다)
    rc = main(argv=['.', 'test', '--exclude', 'signal_vision/vision_hand'])
    assert rc == 0, 'Found code style errors / warnings'

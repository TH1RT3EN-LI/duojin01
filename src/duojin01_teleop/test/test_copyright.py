from ament_copyright.main import main
import pytest


@pytest.mark.skip(reason='No copyright header has been placed in the source file.')
@pytest.mark.copyright
@pytest.mark.linter
def test_copyright():
    assert main(argv=['.', 'test']) == 0, 'Found errors'

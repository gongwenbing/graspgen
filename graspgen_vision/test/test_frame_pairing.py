from graspgen_vision.frame_pairing import FramePairer


def test_reversed_arrival_and_no_cross_frame_pairing():
    p=FramePairer();a=('camera',1,2);b=('camera',1,3)
    assert p.put('objects',a,'meta-a') is None
    assert p.put('cloud',b,'cloud-b') is None
    assert p.put('cloud',a,'cloud-a')==('cloud-a','meta-a')
    assert p.put('objects',b,'meta-b')==('cloud-b','meta-b')
    assert p.put('cloud',a,'old') is None


def test_frame_name_exact_and_cache_bounded():
    p=FramePairer(capacity=2)
    p.put('objects',('camera',1,1),'a')
    assert p.put('cloud',('base_link',1,1),'b') is None
    for i in range(5):p.put('cloud',('camera',2,i),i)
    assert len(p.clouds)==2

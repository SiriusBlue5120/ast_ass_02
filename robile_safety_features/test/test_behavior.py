from robile_safety_features import behaviors


class TestBehaviors:
    
    def test_instantiation():
        bb = behaviors.BatteryStatus2bb()

        assert isinstance(bb, behaviors.BatteryStatus2bb), "Instantiation somehow went wrong?"

    def test_fail_instantiation():
        bb = behaviors.BatteryStatus2bb()

        assert isinstance(bb, behaviors.LaserScan2bb), "What a surprise"
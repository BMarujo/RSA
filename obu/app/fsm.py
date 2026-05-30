try:
    from .car_following import CarFollowingMixin
    from .fsm_core import FsmCoreMixin
    from .fsm_host import HostFsmMixin
    from .fsm_merge import MergeFsmMixin
except ImportError:
    from car_following import CarFollowingMixin
    from fsm_core import FsmCoreMixin
    from fsm_host import HostFsmMixin
    from fsm_merge import MergeFsmMixin


class FsmMixin(FsmCoreMixin, MergeFsmMixin, CarFollowingMixin, HostFsmMixin):
    pass

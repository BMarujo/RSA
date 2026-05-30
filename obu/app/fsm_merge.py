try:
    from .fsm_merge_guards import MergeGuardMixin
    from .fsm_merge_negotiation import MergeNegotiationMixin
    from .fsm_merge_role import MergeRoleMixin
except ImportError:
    from fsm_merge_guards import MergeGuardMixin
    from fsm_merge_negotiation import MergeNegotiationMixin
    from fsm_merge_role import MergeRoleMixin


class MergeFsmMixin(MergeGuardMixin, MergeNegotiationMixin, MergeRoleMixin):
    pass

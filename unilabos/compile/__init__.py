from unilabos.messages import *
from .pump_protocol import generate_pump_protocol, generate_pump_protocol_with_rinsing
from .clean_protocol import generate_clean_protocol
from .separate_protocol import generate_separate_protocol
from .evaporate_protocol import generate_evaporate_protocol
from .evacuateandrefill_protocol import generate_evacuateandrefill_protocol
from .agv_transfer_protocol import generate_agv_transfer_protocol


# Define a dictionary of protocol generators.
action_protocol_generators = {
    PumpTransferProtocol: generate_pump_protocol_with_rinsing,
    CleanProtocol: generate_clean_protocol,
    SeparateProtocol: generate_separate_protocol,
    EvaporateProtocol: generate_evaporate_protocol,
    EvacuateAndRefillProtocol: generate_evacuateandrefill_protocol,
    AGVTransferProtocol: generate_agv_transfer_protocol,
}
# End Protocols

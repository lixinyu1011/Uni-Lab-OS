from pydantic import BaseModel, Field
import pint


class Point3D(BaseModel):
    x: float = Field(..., title="X coordinate")
    y: float = Field(..., title="Y coordinate")
    z: float = Field(..., title="Z coordinate")

# Start Protocols

class PumpTransferProtocol(BaseModel):
    from_vessel: str
    to_vessel: str
    volume: float
    amount: str = ""
    time: float = 0
    viscous: bool = False
    rinsing_solvent: str = "air"
    rinsing_volume: float = 5000
    rinsing_repeats: int = 2
    solid: bool = False
    flowrate: float = 500
    transfer_flowrate: float = 2500


class CleanProtocol(BaseModel):
    vessel: str
    solvent: str
    volume: float
    temp: float
    repeats: int = 1


class SeparateProtocol(BaseModel):
    purpose: str  # 'wash' or 'extract'. 'wash' means that product phase will not be the added solvent phase, 'extract' means product phase will be the added solvent phase. If no solvent is added just use 'extract'.
    product_phase: str # 'top' or 'bottom'. Phase that product will be in.
    from_vessel: str #Contents of from_vessel are transferred to separation_vessel and separation is performed.
    separation_vessel: str # Vessel in which separation of phases will be carried out.
    to_vessel: str # Vessel to send product phase to.
    waste_phase_to_vessel: str # Optional. Vessel to send waste phase to.
    solvent: str # Optional. Solvent to add to separation vessel after contents of from_vessel has been transferred to create two phases.
    solvent_volume: float # Optional. Volume of solvent to add.
    through: str # Optional. Solid chemical to send product phase through on way to to_vessel, e.g. 'celite'.
    repeats: int # Optional. Number of separations to perform.
    stir_time: float # Optional. Time stir for after adding solvent, before separation of phases.
    stir_speed: float # Optional. Speed to stir at after adding solvent, before separation of phases.
    settling_time: float # Optional. Time


class EvaporateProtocol(BaseModel):
    vessel: str
    pressure: float
    temp: float
    time: float
    stir_speed: float


class EvacuateAndRefillProtocol(BaseModel):
    vessel: str
    gas: str
    repeats: int


class AGVTransferProtocol(BaseModel):
    from_repo: dict
    to_repo: dict
    from_repo_position: str
    to_repo_position: str


__all__ = ["Point3D", "PumpTransferProtocol", "CleanProtocol", "SeparateProtocol", "EvaporateProtocol", "EvacuateAndRefillProtocol", "AGVTransferProtocol"]
# End Protocols

# One real, measured surface finding. All vectors are expressed in the
# containing DentalFindingArray.header.frame_id (optical camera frame), SI units.
uint8 ISSUE_UNKNOWN=0
uint8 ISSUE_PLAQUE=1
uint8 ISSUE_STAIN=2
uint8 ISSUE_OTHER=3

uint32 id                  # Unique within this inspection only
uint8 issue_type           # One of the ISSUE_* values above
string tooth_id            # Optional; empty when tooth identity unknown
geometry_msgs/Point position         # Surface point, metres
geometry_msgs/Vector3 surface_normal # Outward unit normal (unitless)
float32 confidence         # Range [0, 1]

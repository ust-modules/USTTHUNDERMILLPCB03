# PCB

Board size: 80.0x80.0 mm (3.15x3.15 inches)

- This is the size of the rectangle that contains the board
- Thickness: 1.6 mm (63 mils)
- Material: FR4
- Finish: Immersion gold
- Layers: 4
- Copper thickness: 35 µm

Solder mask: TOP / BOTTOM

- Color: Green

Silk screen: TOP / BOTTOM

- Color: White

Special features:

- Castellated pads
- Edge plating

Stackup:

| Name                 | Type                 | Color            | Thickness | Material        | Epsilon_r | Loss tangent |
|----------------------|----------------------|------------------|-----------|-----------------|-----------|--------------|
| F.SilkS              | Top Silk Screen      | White            |           |                 |           |              |
| F.Paste              | Top Solder Paste     |                  |           |                 |           |              |
| F.Mask               | Top Solder Mask      | Green            |        10 |                 |           |              |
| F.Cu                 | copper               |                  |        35 |                 |           |              |
| dielectric 1         | core                 |                  |       480 | FR4             |       4.5 |        0.020 |
| In1.Cu               | copper               |                  |        35 |                 |           |              |
| dielectric 2         | prepreg              |                  |       480 | FR4             |       4.5 |        0.020 |
| In2.Cu               | copper               |                  |        35 |                 |           |              |
| dielectric 3         | core                 |                  |       480 | FR4             |       4.5 |        0.020 |
| B.Cu                 | copper               |                  |        35 |                 |           |              |
| B.Mask               | Bottom Solder Mask   | Green            |        10 |                 |           |              |
| B.Paste              | Bottom Solder Paste  |                  |           |                 |           |              |
| B.SilkS              | Bottom Silk Screen   | White            |           |                 |           |              |

# Important sizes

Clearance: 0.2 mm (8 mils)

Track width: 0.2 mm (8 mils)

- By design rules: 0.2 mm (8 mils)

Drill: 0.5 mm (20 mils)

- Vias: 0.5 mm (20 mils) [Design: 0.4 mm (16 mils)]
- Pads: N/A mm (N/A mils)
- The above values are real drill sizes, they add 0.1 mm (4 mils) to plated holes (PTH)

Via: 0.8/0.4 mm (31/16 mils)

- By design rules: 0.4/0.3 mm (16/12 mils)
- Micro via: yes [0.2/0.1 mm (8/4 mils)]
- Buried/blind via: yes
- Total: 220 (thru: 220 buried/blind: 0 micro: 0)

Outer Annular Ring: 0.15 mm (6 mils)

- By design rules: N/A mm (N/A mils)

Eurocircuits class: 4B
- Using min drill 0.5 mm for an OAR of 0.15 mm


# General stats

Components count: (SMD/THT)

- Top: 77/1 (SMD + THT)
- Bottom: 0/4 (THT)

Defined tracks:

- 0.2 mm (8 mils)
- 0.25 mm (10 mils)
- 0.5 mm (20 mils)

Used tracks:

- 0.2 mm (8 mils) (63) defined: yes
- 0.25 mm (10 mils) (289) defined: yes
- 0.3 mm (12 mils) (77) defined: no
- 0.4 mm (16 mils) (2) defined: no
- 0.5 mm (20 mils) (57) defined: yes

Defined vias:


Used vias:

- 0.8/0.4 mm (31/16 mils) (Count: 220, Aspect: 2.0 A) defined: no

Holes (excluding vias):


Oval holes:


Drill tools (including vias and computing adjusts and rounding):

- 0.5 mm (20 mils) (220)





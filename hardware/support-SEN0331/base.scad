$fs = 0.15;

include <params.scad>
include <roundedcube.scad>
difference() {
     roundedcube([x_box, y_box, z_box], false, 0.6, "zmax");
     translate([-1, -1, z_base]) cube([x_box+2, y_box+2, z_box]);
     /* hole for board */
     translate([0, 0, ep_box+0.5]) linear_extrude(height = z_base) import("board.dxf", layer="hole_board");
     translate([d_cable+ep_box, ep_box, z_base-z_board/2]) cube([x_board, y_board, z_board/2+1]);
     /* hole for screws */
     translate([d_cable+ep_box+2, ep_box+2, -1]) cylinder(d=2,h=z_base+2);
     translate([d_cable+ep_box+2, ep_box+2, -1]) cylinder(d=4,h=1+1);
     translate([d_cable+ep_box+2, ep_box+2+15, -1]) cylinder(d=2,h=z_base+2);
     translate([d_cable+ep_box+2, ep_box+2+15, -1]) cylinder(d=4,h=1+1);
     /* hole for cable */
     translate([d_cable/2+ep_box, ep_box+4, z_base]) rotate([-90, 0, 0]) cylinder(d=d_cable, h=y_box+1);
     translate([d_cable/2+ep_box, ep_box+4, z_base-d_cable/2]) cube([d_cable, 11, d_cable/2]);
};

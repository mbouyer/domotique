$fs = 0.15;

include <params.scad>
include <roundedcube.scad>

difference() {
     roundedcube([x_box, y_box, z_cover], false, 0.6, "zmax");
     /* hole for board */
     translate([0, 0, 0]) linear_extrude(height = z_cover - ep_box) import("board.dxf", layer="hole_board");
     translate([d_cable+ep_box, ep_box, -1]) cube([x_board, y_board, z_cover/2+1]);
     /* hole for screws */
     translate([d_cable+ep_box+2, ep_box+2, 0]) cylinder(d=1.5,h=z_cover - ep_box);
     translate([d_cable+ep_box+2, ep_box+2+15, 0]) cylinder(d=1.5,h=z_cover - ep_box);
     /* hole for cable */
     translate([d_cable/2+ep_box, ep_box+4, 0]) rotate([-90, 0, 0]) cylinder(d=d_cable, h=y_box+1);
     translate([d_cable/2+ep_box, ep_box+4, 0]) cube([d_cable, 11, d_cable/2]);
     /* holes for air */
     translate([0, 0, 0]) linear_extrude(height = z_cover + 1) import("board.dxf", layer="hole");
     translate([0, ep_box+1, -y_box/2-4+3]) rotate([90, 0, 0]) linear_extrude(height = y_box/2 + 1) import("board.dxf", layer="hole");

};

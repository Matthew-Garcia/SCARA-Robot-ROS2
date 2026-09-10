// SCARA pickup placement tray — millimetres
// Fits a 256 x 256 mm print bed; print flat with no supports.
$fn = 72;
length = 240;
width = 70;
height = 10;
corner_radius = 5;
pocket_depth = 3.2;

module rounded_plate(l, w, h, r) {
    linear_extrude(height=h)
        hull()
            for (x=[-l/2+r, l/2-r], y=[-w/2+r, w/2-r])
                translate([x,y]) circle(r=r);
}

difference() {
    rounded_plate(length, width, height, corner_radius);
    // Square station for the red cube.
    translate([-90, 0, height-pocket_depth/2+0.1])
        cube([38,38,pocket_depth+0.2], center=true);
    // Round station for the green sphere.
    translate([-30,0,height-pocket_depth])
        cylinder(h=pocket_depth+0.2, r=20);
    // Round station for the yellow cylinder or other pieces.
    translate([30,0,height-pocket_depth])
        cylinder(h=pocket_depth+0.2, r=20);
    // Fourth station for the purple diamond block.
    translate([90, 0, height-pocket_depth/2+0.1])
        rotate([0,0,45]) cube([36,36,pocket_depth+0.2], center=true);
}

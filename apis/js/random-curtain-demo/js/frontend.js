// =====================================================================================================================
// SETTINGS
// =====================================================================================================================

const CAMERA_PARAMS = {
    width: 512,
    height: 640,
    fov: 80.0,
    fps: 60
}

const LASER_PARAMS = {
    displacement: 0.2,
    fov: 45,
    thickness: 0.00055,
    divergence: 0.055,
    maxOmega: 25000,
    maxAlpha: 1.5e7  // 5.0e7
}

const SETTINGS_PARAMS = {
    zMin: 3,
    zMax: 40,
    nodesPerRay: 200,  // 100
    threshold: 0.8,
    rSampling: "linear"
}

// =====================================================================================================================
// Controls Pane
// =====================================================================================================================

const pane = new Tweakpane();

var toggle = pane.addButton({title: 'Controls'})
  .on('click', () => {
    separator.hidden = !separator.hidden;
    cameraPane.hidden = !cameraPane.hidden;
    laserPane.hidden = !laserPane.hidden;
    settingsPane.hidden = !settingsPane.hidden;
    updateButton.hidden = !updateButton.hidden;
  });

const separator = pane.addSeparator();
separator.hidden = true;

const cameraPane = pane.addFolder({
    title: 'Camera',
    expanded: false
});
cameraPane.hidden = true;

cameraPane.addInput(CAMERA_PARAMS, "width", {step: 1, label: "Width"});
cameraPane.addInput(CAMERA_PARAMS, "height", {step: 1, label: "Height"});
cameraPane.addInput(CAMERA_PARAMS, 'fov', {label: "FOV"});
cameraPane.addInput(CAMERA_PARAMS, 'fps', {label: "FPS"});

const laserPane = pane.addFolder({
    title: 'Laser',
    expanded: false
});
laserPane.hidden = true;

laserPane.addInput(LASER_PARAMS, 'displacement', {label: "Separation"});
laserPane.addInput(LASER_PARAMS, 'fov', {label: "FOV"});
laserPane.addInput(LASER_PARAMS, 'thickness', {label: "Thickness"});
laserPane.addInput(LASER_PARAMS, 'divergence', {label: "Divergence"});
laserPane.addInput(LASER_PARAMS, 'maxOmega', {label: "Max Vel."});
laserPane.addInput(LASER_PARAMS, 'maxAlpha', {label: "Max Acc."});

const settingsPane = pane.addFolder({
    title: 'Settings',
    expanded: false
});
settingsPane.hidden = true;

settingsPane.addInput(SETTINGS_PARAMS, 'zMin', {label: "Min Range"});
settingsPane.addInput(SETTINGS_PARAMS, 'zMax', {label: "Max Range"});
settingsPane.addInput(SETTINGS_PARAMS, 'nodesPerRay', {label: "Nodes/Ray", step: 1});
settingsPane.addInput(SETTINGS_PARAMS, 'threshold', {label: "Threshold", min: 0.0, max: 1.0});
settingsPane.addInput(SETTINGS_PARAMS, 'rSampling', {label: "Sampling", options: {Linear: "linear", Uniform: "uniform", Equal: "equal"}});

const updateButton = pane.addButton({title: "Update"});
updateButton.hidden = true;
updateButton.on('click', async () => await reset());

// =====================================================================================================================

function camIntrinsicsMatrix(fovDeg, w, h) {
    var fovRad = fovDeg * Math.PI / 180.0;
    var f = (w / 2) / Math.tan(0.5 * fovRad);
    var cx = w / 2, cy = h / 2;
    var mat = [f, 0., 0., 0., f, 0., cx, cy, 1.]; // column-major
    return mat;
}

// =====================================================================================================================
// Planner initialization
// =====================================================================================================================

function createPlanner() {
    // Camera parameters
    var cparams = new Module.CameraParameters();
    cparams.width = CAMERA_PARAMS.width;
    cparams.height = CAMERA_PARAMS.height;
    cparams.fps = CAMERA_PARAMS.fps;
    var CAM_MATRIX = camIntrinsicsMatrix(CAMERA_PARAMS.fov, CAMERA_PARAMS.width, CAMERA_PARAMS.height);
    cam_matrix_vf = new Module.VectorFloat();
    for (var i = 0; i < CAM_MATRIX.length; i++) {
        cam_matrix_vf.push_back(CAM_MATRIX[i]);
    }
    cparams.cam_matrix = cam_matrix_vf;
    cam_to_laser_vf = new Module.VectorFloat();
    var CAM_TO_LASER=[1., 0., 0., 0., 0., 1., 0., 0., 0., 0., 1., 0., -LASER_PARAMS.displacement, 0., 0., 1.];
    for (var i = 0; i < CAM_TO_LASER.length; i++) {
        cam_to_laser_vf.push_back(CAM_TO_LASER[i]);
    }
    cparams.cam_to_laser = cam_to_laser_vf;

    // Laser parameters
    var lparams = new Module.LaserParameters();
    lparams.fov = LASER_PARAMS.fov;
    lparams.thickness = LASER_PARAMS.thickness;
    lparams.divergence = LASER_PARAMS.divergence;
    lparams.max_omega = LASER_PARAMS.maxOmega;
    lparams.max_alpha = LASER_PARAMS.maxAlpha;

    // ranges
    const ORDER = 1.4;
    var ranges = new Module.VectorFloat();
    for (var i=0; i < SETTINGS_PARAMS.nodesPerRay; i++) {
        var u = (i / SETTINGS_PARAMS.nodesPerRay) ** ORDER;
        ranges.push_back(SETTINGS_PARAMS.zMin + (SETTINGS_PARAMS.zMax - SETTINGS_PARAMS.zMin) * u);
    }

    // interpolator
    var interpolator = new Module.PolarIdentityInterpolator(CAMERA_PARAMS.width, CAMERA_PARAMS.height);

    // planner
    return new Module.PlannerV2(cparams, lparams, ranges, interpolator, false);
}

var canvas = new fabric.Canvas('canvas-tools');;

var planner = null;
var polyLine = null;

async function reset() {
    document.getElementById("loading").style.display = "block";  // display loading div
    
    // setTimeout(resolve, 0) doesn't work sometimes: loading div doesn't display. I don't understand why.
    // setTimeout(resolve, 15) works all the time.
    await new Promise(resolve => setTimeout(resolve, 15));

    // planner
    delete planner;
    planner = createPlanner();

    // polyLine
    delete polyLine;
    polyLine = new PolyLine(canvas);

    document.getElementById("loading").style.display = "none";  // hide loading div
}

Module['onRuntimeInitialized'] = async () => {
    // var person = alert(Module.hello());
    // console.log(Module.lerp(1, 2, 0.5));

    // var interpolator = new Module.PolarIdentityInterpolator(3, 4);
    // var display = "PID(3, 4).isCmapShapeValid(10, 20) : " +
    //               interpolator.isCmapShapeValid(10, 20) + '\n' +
    //               "PID(3, 4).isCmapShapeValid( 4,  3) : " +
    //               interpolator.isCmapShapeValid(4, 3);
    // alert(display);

    await reset();
}

// =================================================================================================================
// Polygon drawing
// =================================================================================================================

function Point(x, y) {
    this.x = x;
    this.y = y;
}

function setResultsMode(mode) {
    var resultsButton = document.getElementById("results_button");
    var resultsDiv = document.getElementById("results_div");
    if (mode == "show") {
        resultsDiv.style.display = "block";
        resultsButton.disabled = false;
        resultsButton.innerText = "Hide Analysis";
    }
    else if (mode == "hide") {
        resultsDiv.style.display = "none";
        resultsButton.disabled = false;
        resultsButton.innerText = "Show Analysis";
    }
    else if (mode == "off") {
        resultsDiv.style.display = "none";
        resultsButton.disabled = true;
        resultsButton.innerText = "No Analysis"
    }
};

var PolyLine = function(canvas) {
    // NOTE: canvas and buttons may already have event listeners that need to be removed

    let self = this;

    self.canvas = canvas;

    // scaling between camera coordinates and canvas.
    // a margin of 1m is used on all four sides.
    self.MARGIN = 1.0;
    self.camXspan = 2.0 * SETTINGS_PARAMS.zMax * Math.sin(0.5 * CAMERA_PARAMS.fov * Math.PI / 180.0) + 2.0 * self.MARGIN;
    self.camZspan = SETTINGS_PARAMS.zMax + 2.0 * self.MARGIN;
    self.SCALE = 800 / Math.max(self.camXspan, self.camZspan);
    self.canvas.setWidth(self.camXspan * self.SCALE);
    self.canvas.setHeight(self.camZspan * self.SCALE);

    self.x = 0;
    self.y = 0;

    self.polyStrokeWidth = 3.0;
    self.poly = null;
    self.polyPoints = [];
    self.lines = [];
    self.lineCounter = 0;
    self.state = "drawing"  // "drawing", "created", "frozen"
    // self.drawingObject = {};
    // self.drawingObject.type = "empty";
    // self.drawingObject.background = "";
    // self.drawingObject.border = "";
    self.tooltip = document.querySelectorAll('.tooltip')[0];

    self.rcLines = [];
    self.rcCircles = [];
    self.chart = null;

    self.clear();
    
    // TODO: do I have to remove this listener?
    fabric.util.addListener(window, 'dblclick', function(){
        if (self.state == "drawing" && self.polyPoints.length > 0) { 
            self.state = "created";
            self.lines.forEach(function(value, index, ar){
                self.canvas.remove(value);
            });
            //canvas.remove(lines[lineCounter - 1]);
            self.poly = self.makepoly();
            self.canvas.add(self.poly);
            self.canvas.renderAll();
            document.getElementById("compute").disabled = false;  // enable compute button
            document.getElementById("instbox").innerHTML = "Move/rotate/resize polygon. When done, click \"<span style='color:red'>Analyze</span>\".";
        }
    });

    self.canvas.off('mouse:down');  // remove previous event listener
    self.canvas.on('mouse:down', function (options) {
        if (self.state == "drawing") {
            self.canvas.selection = false;
            self.setStartingPoint(options); // set x,y
            self.polyPoints.push(new Point(self.x, self.y));
            var points = [self.x, self.y, self.x, self.y];  // line of zero width: endpoint will change on mouse move
            self.lines.push(new fabric.Line(points, {
                strokeWidth: self.polyStrokeWidth,
                selectable: false,
                stroke: '#58c',
                originX: 'left',
                originY: 'top'
            }));
            self.canvas.add(self.lines[self.lineCounter]);
            self.lineCounter++;
            self.canvas.on('mouse:up', function (options) {
                self.canvas.selection = true;
                self.canvas.off('mouse:up');
                document.getElementById("instbox").innerText = "Single-click to add edge. Double-click to close polygon.";
            });
        }
    });

    self.canvas.off('mouse:move');  // remove previous event listener
    self.canvas.on('mouse:move', function (options) {
        self.setStartingPoint(options); // set x,y
        self.tooltip.style.left = options.e.pageX + 10 + 'px';
        self.tooltip.style.top  = options.e.pageY + 10 + 'px';
        self.tooltip.style.visibility = "visible";
        let { xc, zc } = self.pix2cam(self.x, self.y);
        self.tooltip.innerText = "x: " + xc.toFixed(1) + ", z: " + zc.toFixed(1);
        if (self.lines[0] !== null && self.lines[0] !== undefined && self.state == "drawing") {
            self.setStartingPoint(options);
            self.lines[self.lineCounter - 1].set({
                x2: self.x,
                y2: self.y
            });
            self.canvas.renderAll();
        }
    });
    
    $("#poly").off('click');  // remove previous event listener
    $("#poly").click(function () {
        self.clear();
    });

    $("#random_curtain").off('click');  // remove previous event listener
    $("#random_curtain").click(function() {
        // random curtain from planner
        var curtain = planner.randomCurtainDiscreteJs(SETTINGS_PARAMS.rSampling);

        // process points
        var points = [];
        for (var i = 0; i < curtain.size(); i++) {
            var pt = curtain.get(i);
            var xc = pt.get(0), zc = pt.get(1), intensity = pt.get(2);
            var loc = self.cam2pix(xc, zc);
            points.push([loc, intensity]);
        };

        // remove previous rcLines and rcCircles
        for (var line of self.rcLines) {
            self.canvas.remove(line);
        }
        for (var circle of self.rcCircles) {
            self.canvas.remove(circle);
        }
        

        self.rcLines = [];
        self.rcCircles = [];
        for (var i = 0; i < points.length; i++) {
            var currPt = points[i][0];
            var intensity = points[i][1];

            // create circles
            if (intensity > SETTINGS_PARAMS.threshold) {
                var circle = new fabric.Circle({
                    left: currPt.xp,  // center point
                    top: currPt.yp,   // center point
                    radius: 3,
                    fill: "yellow",
                    stroke: "black",
                    strokeWidth: 1,
                    originX: "center",
                    originY: "center",
                    hasControls: false,
                    hasBorders: false,
                    selectable: false
                });
                self.rcCircles.push(circle);
            }

            // create lines
            if (i < points.length - 1) {
                var nextPt = points[i+1][0];
                var line = new fabric.Line([currPt.xp, currPt.yp, nextPt.xp, nextPt.yp], {
                    selectable: false,
                    stroke: 'rgba(255, 155, 165, 1)',  // "pink"
                    strokeWidth: 1.5,
                    originX: 'center',
                    originY: 'center'
                });
                self.rcLines.push(line);
            }
        }

        // add rcLines and rcCircles to canvas
        for (var line of self.rcLines) {
            self.canvas.add(line);
            canvas.sendToBack(line);
        }
        for (var circle of self.rcCircles) {
            self.canvas.add(circle);
        }

        self.canvas.renderAll();
        if (self.state == "frozen")
            document.getElementById("instbox").innerHTML = "Click \"<span style='color:red'>Show/Hide Analysis</span>\" to view analysis report.";
    });

    $("#compute").off('click');  // remove previous event listener
    $("#compute").click(async function() {
        if (self.state == "created") {

            // Freeze poly.
            self.state = "frozen";
            
            // Remove poly.
            self.canvas.remove(self.poly);

            // Update polyPoints.
            // code from: https://stackoverflow.com/a/53710375
            var matrix = self.poly.calcTransformMatrix();
            
            // Previously: polyPoints were according to (left, top) origin when stored in the mouse:down function.
            //             Hence, lines were pushed according to (left, top).
            // Now: the output of self.poly.get("points") followed by the two transforms is the current vertices
            //      according to (center, center) origin, which is what all objects on the canvas should be rendered
            //      with.
            self.polyPoints = self.poly.get("points")
                .map(function(p){
                    return new fabric.Point(
                        p.x - self.poly.pathOffset.x,
                        p.y - self.poly.pathOffset.y
                    );
                })
                .map(function(p){
                    return fabric.util.transformPoint(p, matrix);
                });

            //  Replace poly with blue solid lines.
            for (var i = 0; i < self.polyPoints.length - 1; i++) {
                var sPoint = self.polyPoints[i];
                var ePoint = self.polyPoints[i+1];
                var points = [sPoint.x, sPoint.y, ePoint.x, ePoint.y];
                var line = new fabric.Line(points, {
                    strokeWidth: self.polyStrokeWidth,
                    selectable: false,
                    stroke: 'blue',
                    originX: 'center',
                    originY: 'center'
                });
                self.canvas.add(line);
            }

            // Compute visible depth points.

            // Create lineSurface input.
            var lineSurface = new Module.VectorVectorFloat();
            for (var i = 0; i < self.polyPoints.length; i++) {
                var p1p = self.polyPoints[i];
                var p1c = self.pix2cam(p1p.x, p1p.y);

                var p2p = self.polyPoints[(i + 1) % self.polyPoints.length];
                var p2c = self.pix2cam(p2p.x, p2p.y);
                
                var segment = new Module.VectorFloat();
                for (var val of [p1c.xc, p1c.zc, p2c.xc, p2c.zc]) {
                    segment.push_back(val);
                }
                
                lineSurface.push_back(segment);
            }

            // Call planner functions.
            planner.loadLineSurfaceJs(lineSurface);
            planner.computeVisibleRanges();
            var points = planner.visiblePointsJs();

            // Plot points as circles.
            for (var i = 0; i < points.size(); i++) {
                var point = points.get(i);
                var xc = point.get(0), zc = point.get(1);
                let {xp, yp} = self.cam2pix(xc, zc);
            
                var circle = new fabric.Circle({
                    left: xp,  // center point
                    top: yp,   // center point
                    radius: 0.6 * self.polyStrokeWidth,
                    fill: "red",
                    originX: "center",
                    originY: "center",
                    hasControls: false,
                    hasBorders: false,
                    selectable: false
                });
                
                self.canvas.add(circle);
            }

            // compute intensities
            planner.computeLayoutIntensities();

            self.canvas.renderAll();
            document.getElementById("results_button").innerText = "Analyzing ...";

            // Push the rest of the code in this function to the event loop so that the browser can render.
            // Discussion: https://stackoverflow.com/q/779379
            // Explanation: https://youtu.be/8aGhZQkoFbQ
            await new Promise(response => setTimeout(response, 0));

            // compute hit probability
            var hitProb = planner.randomCurtainHitProb(SETTINGS_PARAMS.threshold, SETTINGS_PARAMS.rSampling);
            drawResults(hitProb);
            setResultsMode("hide");
            document.getElementById("instbox").innerHTML = "Click \"<span style='color:red'>Random Curtain</span>\" to generate random curtains & hits."

            // disable compute button
            this.disabled = true;
        }
    });

    function drawResults(p) {
        // Probability bar.
        var pcentStr = (100 * p).toFixed(1) + '%';
        var pbarColor;
        if (p < 0.333) pbarColor = "red";
        else if (p < 0.666) pbarColor = "yellow";
        else pbarColor = "lime";
        
        document.getElementById("probability_bar").style.width = pcentStr;
        document.getElementById("probability_bar").style.backgroundColor = pbarColor;
        document.getElementById("probabilty_number").innerText = pcentStr;

        // Expected number of light curtains.
        var expectedNumber = 1.0 / p;
        document.getElementById("expected_number").innerText = expectedNumber.toFixed(3);

        // Expected time.
        var expectedTime = expectedNumber / 60.0 * 1000;
        document.getElementById("expected_time").innerText = expectedTime.toFixed(1) + " ms";

        // Plot.
        var times = [];
        var probs = [];
        for (var numCurtains of [1, 2, 3, 4, 5, 6, 7, 8, 9, 10]) {
            var time = (numCurtains * 1000 / 60).toFixed(0);
            var prob = 1.0 - (1.0 - p) ** numCurtains;
            times.push(time);
            probs.push(prob);
        }
        var ctx = document.getElementById('plot').getContext('2d');
        if (self.chart != null) self.chart.destroy();
        self.chart = new Chart(ctx, {
            // The type of chart we want to create
            type: 'line',
        
            // The data for our dataset
            data: {
                labels: times,
                datasets: [{
                    // backgroundColor: 'rgb(255, 99, 132)',
                    borderColor: pbarColor,
                    data: probs
                }]
            },
        
            // Configuration options go here
            options: {
                legend: {
                    display: false
                },
                scales: {
                    xAxes: [{scaleLabel: {display: true, labelString: "Time (ms)"}}],
                    yAxes: [{scaleLabel: {display: true, labelString: "Probability"}}]
                }
            }
        });
    }


    $("#results_button").off('click');  // remove previous event listener
    $("#results_button").click(function () {
        var resultsDiv = document.getElementById("results_div");
        if (resultsDiv.style.display === "none") {
            setResultsMode("show");
        }
        else {
            setResultsMode("hide");
        }
    });
}

PolyLine.prototype = {
    addAnnos: function() {
        let self = this;
        var size = 1;

        // legend: rectangle
        self.canvas.add(new fabric.Rect({
            selectable: false,
            left: self.MARGIN * self.SCALE,
            top: self.MARGIN * self.SCALE,
            width: size * self.SCALE,
            height: size * self.SCALE,
            fill: '',
            stroke: 'red',
            strokeWidth: 1.5,
            originX: 'left',
            originY: 'top'
        }));

        // legend: text
        var size_text = size.toString();
        self.canvas.add(new fabric.Text(size_text + "m x " + size_text + "m", {
            selectable: false,
            left: self.MARGIN * self.SCALE,
            top: (self.MARGIN + size + 0.5) * self.SCALE,
            fontSize: 15,
            originX: 'left',
            originY: 'top'
        }));

        // laser
        var radius = 0.4;  // in meters
        var point = self.cam2pix(LASER_PARAMS.displacement, 0.0);
        self.canvas.add(new fabric.Circle({
            selectable: false,
            radius: radius * self.SCALE,
            left: point.xp,
            top: point.yp,
            fill: 'green',
            originX: 'center',
            originY: 'center'
        }));

        // camera
        var point = self.cam2pix(0.0, 0.0);
        self.canvas.add(new fabric.Circle({
            selectable: false,
            radius: radius * self.SCALE,
            left: point.xp,
            top: point.yp,
            fill: 'red',
            originX: 'center',
            originY: 'center'
        }));

        // rays
        var origin = self.cam2pix(0.0, 0.0);
        var halfAngle = 0.5 * CAMERA_PARAMS.fov * Math.PI / 180.0;
        var leftPt = self.cam2pix(-SETTINGS_PARAMS.zMax * Math.sin(halfAngle), SETTINGS_PARAMS.zMax * Math.cos(halfAngle));
        var rightPt = self.cam2pix(SETTINGS_PARAMS.zMax * Math.sin(halfAngle), SETTINGS_PARAMS.zMax * Math.cos(halfAngle));

        // left ray
        self.canvas.add(new fabric.Line([origin.xp, origin.yp, leftPt.xp, leftPt.yp], {
            selectable: false,
            stroke: 'red',
            strokeWidth: 1.5,
            originX: 'center',
            originY: 'center'
        }));

        // right ray
        self.canvas.add(new fabric.Line([origin.xp, origin.yp, rightPt.xp, rightPt.yp], {
            selectable: false,
            stroke: 'red',
            strokeWidth: 1.5,
            originX: 'center',
            originY: 'center'
        }));

        // arc
        self.canvas.add(new fabric.Circle({
            selectable: false,
            radius: SETTINGS_PARAMS.zMax * self.SCALE,
            left: origin.xp,
            top: origin.yp,
            startAngle: (270 - 0.5 * CAMERA_PARAMS.fov) * Math.PI / 180.0,
            endAngle: (270 + 0.5 * CAMERA_PARAMS.fov) * Math.PI / 180.0,
            fill: '',
            stroke: 'red',
            originX: 'center',
            originY: 'center'
        }));
    },

    clear: function() {
        let self = this;
        self.canvas.clear();
        self.poly = null;
        self.polyPoints.length = 0;
        self.lines.length = 0;
        self.lineCounter = 0;
        self.state = "drawing";
        planner.clear();
        // self.drawingObject = {};
        // self.drawingObject.type = "empty";
        self.addAnnos();
        self.canvas.renderAll();
        setResultsMode("off");
        document.getElementById("instbox").innerText = "Single-click on canvas to start drawing.";
    },

    pix2cam: function(xp, yp) {
        var self = this;
        var xc = (xp / self.SCALE) - 0.5 * self.camXspan;
        var zc = self.camZspan - self.MARGIN - (yp / self.SCALE);
        return { xc: xc, zc: zc };
    },

    cam2pix: function(xc, zc) {
        var self = this;
        var xp = (xc + 0.5 * self.camXspan) * self.SCALE;
        var yp = (self.camZspan - self.MARGIN - zc) * self.SCALE;
        return {xp: xp, yp: yp};
    },

    findTopPaddingForpoly: function() {
        let self = this;
        var result = 999999;
        for (var f = 0; f < self.lineCounter; f++) {
            if (self.polyPoints[f].y < result) {
                result = self.polyPoints[f].y;
            }
        }
        return Math.abs(result);
    },

    findLeftPaddingForpoly: function() {
        let self = this;
        var result = 999999;
        for (var i = 0; i < self.lineCounter; i++) {
            if (self.polyPoints[i].x < result) {
                result = self.polyPoints[i].x;
            }
        }
        return Math.abs(result);
    },

    makepoly: function() {
        let self = this;
        var left = self.findLeftPaddingForpoly();
        var top  = self.findTopPaddingForpoly();
        self.polyPoints[self.polyPoints.length - 1] = new Point(self.polyPoints[0].x, self.polyPoints[0].y);
        // self.polyPoints.push(new Point(self.polyPoints[0].x, self.polyPoints[0].y))
        var poly = new fabric.Polyline(self.polyPoints, {
            fill: 'rgba(0,0,0,0)',
            stroke:'#58c',
            strokeWidth: self.polyStrokeWidth,
            strokeUniform: true
        });

        poly.set({    
            left: left,
            top: top
        });

        // poly.on("modified", function () {
        //     // code from: https://stackoverflow.com/a/53710375
        //     var matrix = this.calcTransformMatrix();
        //     self.polyPoints = this.get("points")
        //         .map(function(p){
        //             return new fabric.Point(
        //                 p.x - poly.pathOffset.x,
        //                 p.y - poly.pathOffset.y
        //             );
        //         })
        //         .map(function(p){
        //             return fabric.util.transformPoint(p, matrix);
        //         });

        //     // display circles for debugging
        //     // var circles = []
        //     // for (var i = 0; i < self.polyPoints.length - 1; i++) {
        //     //     var p = self.polyPoints[i];
        //     //     var circle = new fabric.Circle({
        //     //         left: p.x,
        //     //         top: p.y,
        //     //         radius: 3,
        //     //         fill: "red",
        //     //         originX: "center",
        //     //         originY: "center",
        //     //         hasControls: false,
        //     //         hasBorders: false,
        //     //         selectable: false
        //     //       });
        //     //     // self.canvas.add(circle);
        //     //     circles.push(circle);
        //     // }
        //     // self.canvas.clear().add(self.poly).add.apply(self.canvas, circles).setActiveObject(self.poly).renderAll();

        // });

        return poly;
    },

    setStartingPoint: function(options) {
        let self = this;
        var offset = $('#canvas-tools').offset();
        self.x = options.e.pageX - offset.left;
        self.y = options.e.pageY - offset.top;
    }
}

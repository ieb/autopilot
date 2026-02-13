/**
 * Pogo 12.50 Sailboat Model
 *
 * Procedural WebGL geometry for the Pogo 1250 hull, deck, cabin, cockpit,
 * mast, boom, and standing rigging. Derived from original Finot-Conq plan
 * drawings.
 *
 * Coordinate system: X = port(-)/starboard(+), Y = bow(-)/stern(+), Z = up(+)
 * Scale: 0.16 units per metre (hull LOA = 2.0 units)
 *
 * Key real dimensions:
 *   LOA: 12.5m, Beam: 3.94m, Freeboard: ~1.0m
 *   P (mast luff): 15.705m, E (boom): 5.4m
 *   I (forestay height): 15.14m, J (foretriangle base): 3.395m
 */

// =============================================================================
// Boat Mesh (triangulated surfaces)
// =============================================================================

function createPogo1250Buffers(gl) {
    const S = 0.16;  // model scale: units per metre

    // Geometry arrays (built up, then converted to typed arrays)
    const positions = [];
    const colors = [];
    const indices = [];

    function addVert(x, y, z, r, g, b) {
        const idx = positions.length / 3;
        positions.push(x, y, z);
        colors.push(r, g, b);
        return idx;
    }

    function addTri(a, b, c) {
        indices.push(a, b, c);
    }

    function addQuad(a, b, c, d) {
        indices.push(a, b, c, a, c, d);
    }

    // =========================================================================
    // Colours
    // =========================================================================
    const HULL_UPPER  = [0.93, 0.93, 0.91];
    const HULL_LOWER  = [0.88, 0.88, 0.86];
    const DECK_COL    = [0.75, 0.75, 0.72];
    const BOTTOM_COL  = [0.15, 0.15, 0.18];
    const TRANSOM_COL = [0.90, 0.90, 0.88];
    const CABIN_COL   = [0.80, 0.80, 0.78];
    const COCKPIT_COL = [0.50, 0.50, 0.48];
    const SPAR_BLACK  = [0.08, 0.08, 0.10];

    // =========================================================================
    // Hull dimensions
    // =========================================================================
    const FREEBOARD  = S * 1.0;       // 0.16 units
    const CHINE_Z    = FREEBOARD * 0.40;
    const CHINE_STEP = 1.04;          // 4% outward step at chine

    // Hull stations: Y position (bow=-1, stern=+1) and half-beam at deck
    // Derived from the plan-view drawing of the deck outline
    // Traced from Finot-Conq plan view drawing.
    // The Pogo 12.50 has nearly linear widening from bow to max beam at ~75%
    // of LOA, with beam held through to the wide transom.
    const stations = [
        { y: -1.00, hb: 0.000 },   // bow tip
        { y: -0.92, hb: 0.018 },   // bow rounding
        { y: -0.80, hb: 0.050 },   //  10% from bow
        { y: -0.65, hb: 0.090 },   //  18% from bow
        { y: -0.50, hb: 0.130 },   //  25% from bow
        { y: -0.35, hb: 0.168 },   //  33% from bow
        { y: -0.20, hb: 0.205 },   //  40% from bow (mast region)
        { y: -0.05, hb: 0.238 },   //  48% from bow
        { y:  0.10, hb: 0.268 },   //  55% from bow
        { y:  0.25, hb: 0.293 },   //  63% from bow
        { y:  0.40, hb: 0.310 },   //  70% from bow
        { y:  0.50, hb: 0.315 },   //  75% from bow — max beam
        { y:  0.65, hb: 0.315 },   //  beam held
        { y:  0.80, hb: 0.315 },   //  aft
        { y:  1.00, hb: 0.315 },   //  transom
    ];

    // =========================================================================
    // Hull side vertices (6 per station: deck/chine/waterline x port/starboard)
    // =========================================================================
    const stnBase = [];  // base vertex index for each station

    for (let i = 0; i < stations.length; i++) {
        const { y, hb } = stations[i];
        const chb = hb * CHINE_STEP;
        const base = positions.length / 3;
        stnBase.push(base);

        if (hb < 0.001) {
            // Bow tip: all vertices collapse to centreline
            addVert(0, y, FREEBOARD, ...HULL_UPPER);   // 0: deck port
            addVert(0, y, FREEBOARD, ...HULL_UPPER);   // 1: deck starboard
            addVert(0, y, CHINE_Z,   ...HULL_LOWER);   // 2: chine port
            addVert(0, y, CHINE_Z,   ...HULL_LOWER);   // 3: chine starboard
            addVert(0, y, 0,         ...BOTTOM_COL);    // 4: waterline port
            addVert(0, y, 0,         ...BOTTOM_COL);    // 5: waterline starboard
        } else {
            addVert(-hb,  y, FREEBOARD, ...HULL_UPPER); // 0: deck port
            addVert( hb,  y, FREEBOARD, ...HULL_UPPER); // 1: deck starboard
            addVert(-chb, y, CHINE_Z,   ...HULL_LOWER); // 2: chine port
            addVert( chb, y, CHINE_Z,   ...HULL_LOWER); // 3: chine starboard
            addVert(-chb, y, 0,         ...BOTTOM_COL);  // 4: waterline port
            addVert( chb, y, 0,         ...BOTTOM_COL);  // 5: waterline starboard
        }
    }

    // =========================================================================
    // Hull side panels (quad strips between adjacent stations)
    // =========================================================================
    for (let i = 0; i < stations.length - 1; i++) {
        const a = stnBase[i];
        const b = stnBase[i + 1];

        // Starboard upper (deck to chine)
        addQuad(a + 1, a + 3, b + 3, b + 1);
        // Starboard lower (chine to waterline)
        addQuad(a + 3, a + 5, b + 5, b + 3);
        // Port upper (deck to chine)
        addQuad(a + 0, b + 0, b + 2, a + 2);
        // Port lower (chine to waterline)
        addQuad(a + 2, b + 2, b + 4, a + 4);
    }

    // =========================================================================
    // Deck surface (centreline strip triangulation)
    // =========================================================================
    const deckCentre = [];
    for (let i = 0; i < stations.length; i++) {
        deckCentre.push(addVert(0, stations[i].y, FREEBOARD, ...DECK_COL));
    }

    for (let i = 0; i < stations.length - 1; i++) {
        const c0 = deckCentre[i];
        const c1 = deckCentre[i + 1];
        const p0 = stnBase[i];          // deck port
        const s0 = stnBase[i] + 1;      // deck starboard
        const p1 = stnBase[i + 1];
        const s1 = stnBase[i + 1] + 1;

        // Starboard half
        addQuad(c0, s0, s1, c1);
        // Port half
        addQuad(c0, c1, p1, p0);
    }

    // =========================================================================
    // Bottom surface (flat at waterline Z=0)
    // =========================================================================
    const btmCentre = [];
    for (let i = 0; i < stations.length; i++) {
        btmCentre.push(addVert(0, stations[i].y, 0, ...BOTTOM_COL));
    }

    for (let i = 0; i < stations.length - 1; i++) {
        const c0 = btmCentre[i];
        const c1 = btmCentre[i + 1];
        const p0 = stnBase[i] + 4;
        const s0 = stnBase[i] + 5;
        const p1 = stnBase[i + 1] + 4;
        const s1 = stnBase[i + 1] + 5;

        addQuad(c0, p0, p1, c1);
        addQuad(c0, c1, s1, s0);
    }

    // =========================================================================
    // Transom (flat stern face)
    // =========================================================================
    {
        const ls = stations[stations.length - 1];
        const chb = ls.hb * CHINE_STEP;

        const tp  = addVert(-ls.hb, ls.y, FREEBOARD, ...TRANSOM_COL);
        const ts  = addVert( ls.hb, ls.y, FREEBOARD, ...TRANSOM_COL);
        const tcp = addVert(-chb,   ls.y, CHINE_Z,   ...TRANSOM_COL);
        const tcs = addVert( chb,   ls.y, CHINE_Z,   ...TRANSOM_COL);
        const twp = addVert(-chb,   ls.y, 0,         ...TRANSOM_COL);
        const tws = addVert( chb,   ls.y, 0,         ...TRANSOM_COL);

        addQuad(tp, ts, tcs, tcp);    // upper transom
        addQuad(tcp, tcs, tws, twp);  // lower transom
    }

    // =========================================================================
    // Cabin top (raised coachroof, forward of mast)
    // =========================================================================
    {
        const fwd = -0.58;
        const aft = -0.38;
        const hw  =  0.088;                    // half-width
        const topZ = FREEBOARD + S * 0.25;     // cabin top height
        const dkZ  = FREEBOARD;

        // Top face
        const t0 = addVert(-hw, fwd, topZ, ...CABIN_COL);
        const t1 = addVert( hw, fwd, topZ, ...CABIN_COL);
        const t2 = addVert( hw, aft, topZ, ...CABIN_COL);
        const t3 = addVert(-hw, aft, topZ, ...CABIN_COL);
        addQuad(t0, t1, t2, t3);

        // Forward face
        const ff0 = addVert(-hw, fwd, dkZ, ...CABIN_COL);
        const ff1 = addVert( hw, fwd, dkZ, ...CABIN_COL);
        const ff2 = addVert( hw, fwd, topZ, ...CABIN_COL);
        const ff3 = addVert(-hw, fwd, topZ, ...CABIN_COL);
        addQuad(ff0, ff1, ff2, ff3);

        // Aft face
        const af0 = addVert( hw, aft, dkZ, ...CABIN_COL);
        const af1 = addVert(-hw, aft, dkZ, ...CABIN_COL);
        const af2 = addVert(-hw, aft, topZ, ...CABIN_COL);
        const af3 = addVert( hw, aft, topZ, ...CABIN_COL);
        addQuad(af0, af1, af2, af3);

        // Port side
        const ps0 = addVert(-hw, fwd, dkZ, ...CABIN_COL);
        const ps1 = addVert(-hw, aft, dkZ, ...CABIN_COL);
        const ps2 = addVert(-hw, aft, topZ, ...CABIN_COL);
        const ps3 = addVert(-hw, fwd, topZ, ...CABIN_COL);
        addQuad(ps0, ps3, ps2, ps1);

        // Starboard side
        const ss0 = addVert( hw, fwd, dkZ, ...CABIN_COL);
        const ss1 = addVert( hw, aft, dkZ, ...CABIN_COL);
        const ss2 = addVert( hw, aft, topZ, ...CABIN_COL);
        const ss3 = addVert( hw, fwd, topZ, ...CABIN_COL);
        addQuad(ss0, ss1, ss2, ss3);
    }

    // =========================================================================
    // Cockpit well (depressed area, aft of mast)
    // =========================================================================
    {
        const fwd = -0.25;
        const aft =  0.25;
        const hw  = 0.126;                      // half-width (40% of max half-beam)
        const flZ = FREEBOARD - S * 0.20;       // cockpit floor
        const dkZ = FREEBOARD;

        // Floor (faces up)
        const fl0 = addVert(-hw, fwd, flZ, ...COCKPIT_COL);
        const fl1 = addVert( hw, fwd, flZ, ...COCKPIT_COL);
        const fl2 = addVert( hw, aft, flZ, ...COCKPIT_COL);
        const fl3 = addVert(-hw, aft, flZ, ...COCKPIT_COL);
        addQuad(fl0, fl1, fl2, fl3);

        // Forward wall
        const fw0 = addVert(-hw, fwd, flZ, ...COCKPIT_COL);
        const fw1 = addVert( hw, fwd, flZ, ...COCKPIT_COL);
        const fw2 = addVert( hw, fwd, dkZ, ...COCKPIT_COL);
        const fw3 = addVert(-hw, fwd, dkZ, ...COCKPIT_COL);
        addQuad(fw0, fw3, fw2, fw1);

        // Aft wall
        const aw0 = addVert(-hw, aft, flZ, ...COCKPIT_COL);
        const aw1 = addVert( hw, aft, flZ, ...COCKPIT_COL);
        const aw2 = addVert( hw, aft, dkZ, ...COCKPIT_COL);
        const aw3 = addVert(-hw, aft, dkZ, ...COCKPIT_COL);
        addQuad(aw0, aw1, aw2, aw3);

        // Port wall
        const pw0 = addVert(-hw, fwd, flZ, ...COCKPIT_COL);
        const pw1 = addVert(-hw, aft, flZ, ...COCKPIT_COL);
        const pw2 = addVert(-hw, aft, dkZ, ...COCKPIT_COL);
        const pw3 = addVert(-hw, fwd, dkZ, ...COCKPIT_COL);
        addQuad(pw0, pw1, pw2, pw3);

        // Starboard wall
        const sw0 = addVert( hw, fwd, flZ, ...COCKPIT_COL);
        const sw1 = addVert( hw, aft, flZ, ...COCKPIT_COL);
        const sw2 = addVert( hw, aft, dkZ, ...COCKPIT_COL);
        const sw3 = addVert( hw, fwd, dkZ, ...COCKPIT_COL);
        addQuad(sw0, sw3, sw2, sw1);
    }

    // =========================================================================
    // Mast (oval extrusion, 16 segments)
    // Major 300mm fore-aft (Y), minor 200mm athwartships (X)
    // =========================================================================
    {
        const mastY    = -1.0 + S * 6.01;       // mast step 5m from bow = Y -0.20
        const baseZ    = FREEBOARD;
        const topZ     = FREEBOARD + S * 16.705;
        const rx       = S * 0.10;             // minor radius (X) = 100mm
        const ry       = S * 0.15;             // major radius (Y) = 150mm
        const segs     = 16;

        // Bottom ring
        const botRing = [];
        for (let i = 0; i < segs; i++) {
            const ang = (i / segs) * Math.PI * 2;
            botRing.push(addVert(
                Math.cos(ang) * rx,
                mastY + Math.sin(ang) * ry,
                baseZ,
                ...SPAR_BLACK
            ));
        }

        // Top ring
        const topRing = [];
        for (let i = 0; i < segs; i++) {
            const ang = (i / segs) * Math.PI * 2;
            topRing.push(addVert(
                Math.cos(ang) * rx,
                mastY + Math.sin(ang) * ry,
                topZ,
                ...SPAR_BLACK
            ));
        }

        // Side faces
        for (let i = 0; i < segs; i++) {
            const j = (i + 1) % segs;
            addQuad(botRing[i], botRing[j], topRing[j], topRing[i]);
        }

        // Bottom cap
        const botC = addVert(0, mastY, baseZ, ...SPAR_BLACK);
        for (let i = 0; i < segs; i++) {
            addTri(botC, botRing[(i + 1) % segs], botRing[i]);
        }

        // Top cap
        const topC = addVert(0, mastY, topZ, ...SPAR_BLACK);
        for (let i = 0; i < segs; i++) {
            addTri(topC, topRing[i], topRing[(i + 1) % segs]);
        }
    }

    // =========================================================================
    // Boom (rounded-rectangle extrusion along Y)
    // 250mm high (Z), 150mm wide (X), shiny black
    // =========================================================================
    {
        const mastY     = -1.0 + S * 6.01;
        const boomStart = mastY;
        const boomEnd   = mastY + S * 5.4;
        const boomZ     = FREEBOARD + S * 1.2;  // gooseneck ~1.2m above deck
        const hw        = S * 0.075;             // half-width  (X) = 75mm
        const hh        = S * 0.125;             // half-height (Z) = 125mm
        const cr        = Math.min(hw, hh) * 0.4;
        const segsPC    = 3;                     // segments per corner

        // Build rounded-rect cross-section profile
        const profile = [];
        const corners = [
            { cx:  hw - cr, cz:  hh - cr, sa: 0 },
            { cx: -hw + cr, cz:  hh - cr, sa: Math.PI / 2 },
            { cx: -hw + cr, cz: -hh + cr, sa: Math.PI },
            { cx:  hw - cr, cz: -hh + cr, sa: Math.PI * 1.5 },
        ];
        for (const c of corners) {
            for (let i = 0; i <= segsPC; i++) {
                const ang = c.sa + (i / segsPC) * (Math.PI / 2);
                profile.push({
                    x: c.cx + Math.cos(ang) * cr,
                    z: c.cz + Math.sin(ang) * cr
                });
            }
        }
        const nPts = profile.length;

        // Forward ring (at mast)
        const fwdRing = profile.map(p =>
            addVert(p.x, boomStart, boomZ + p.z, ...SPAR_BLACK)
        );

        // Aft ring (boom end)
        const aftRing = profile.map(p =>
            addVert(p.x, boomEnd, boomZ + p.z, ...SPAR_BLACK)
        );

        // Side faces
        for (let i = 0; i < nPts; i++) {
            const j = (i + 1) % nPts;
            addQuad(fwdRing[i], fwdRing[j], aftRing[j], aftRing[i]);
        }

        // Forward cap
        const fwdC = addVert(0, boomStart, boomZ, ...SPAR_BLACK);
        for (let i = 0; i < nPts; i++) {
            addTri(fwdC, fwdRing[(i + 1) % nPts], fwdRing[i]);
        }

        // Aft cap
        const aftC = addVert(0, boomEnd, boomZ, ...SPAR_BLACK);
        for (let i = 0; i < nPts; i++) {
            addTri(aftC, aftRing[i], aftRing[(i + 1) % nPts]);
        }
    }

    // =========================================================================
    // Bowsprit (black carbon fibre pole extending forward from bow)
    // ~1.8m long, ~90mm diameter circular tube
    // =========================================================================
    {
        const bowY      = -1.0;                    // bow of hull (forestay attachment)
        const spritLen  = S * 1.8;                 // bowsprit length
        const spritEnd  = bowY - spritLen;          // tip of bowsprit
        const spritZ    = FREEBOARD;                // horizontal at deck level
        const r         = S * 0.045;                // radius ~45mm
        const segs      = 8;

        // Aft ring (at bow)
        const bsAftRing = [];
        for (let i = 0; i < segs; i++) {
            const ang = (i / segs) * Math.PI * 2;
            bsAftRing.push(addVert(
                Math.cos(ang) * r,
                bowY + Math.sin(ang) * r,
                spritZ,
                ...SPAR_BLACK
            ));
        }

        // Forward ring (at bowsprit tip)
        const bsFwdRing = [];
        for (let i = 0; i < segs; i++) {
            const ang = (i / segs) * Math.PI * 2;
            bsFwdRing.push(addVert(
                Math.cos(ang) * r,
                spritEnd + Math.sin(ang) * r,
                spritZ,
                ...SPAR_BLACK
            ));
        }

        // Side faces
        for (let i = 0; i < segs; i++) {
            const j = (i + 1) % segs;
            addQuad(bsAftRing[i], bsAftRing[j], bsFwdRing[j], bsFwdRing[i]);
        }

        // Aft cap
        const bsAftC = addVert(0, bowY, spritZ, ...SPAR_BLACK);
        for (let i = 0; i < segs; i++) {
            addTri(bsAftC, bsAftRing[(i + 1) % segs], bsAftRing[i]);
        }

        // Forward cap
        const bsFwdC = addVert(0, spritEnd, spritZ, ...SPAR_BLACK);
        for (let i = 0; i < segs; i++) {
            addTri(bsFwdC, bsFwdRing[i], bsFwdRing[(i + 1) % segs]);
        }
    }

    // =========================================================================
    // Create WebGL buffers
    // =========================================================================
    const posArr = new Float32Array(positions);
    const colArr = new Float32Array(colors);
    const idxArr = new Uint16Array(indices);

    const positionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, posArr, gl.STATIC_DRAW);

    const colorBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, colArr, gl.STATIC_DRAW);

    const indexBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ELEMENT_ARRAY_BUFFER, indexBuffer);
    gl.bufferData(gl.ELEMENT_ARRAY_BUFFER, idxArr, gl.STATIC_DRAW);

    console.log('Pogo 1250 model: ' + (posArr.length / 3) + ' vertices, ' +
                (idxArr.length / 3) + ' triangles');

    return {
        position: positionBuffer,
        color: colorBuffer,
        indices: indexBuffer,
        numIndices: idxArr.length
    };
}

// =============================================================================
// Standing Rigging (GL_LINES)
// =============================================================================

function createRiggingBuffers(gl) {
    const S = 0.16;
    const positions = [];
    const colors = [];

    function addLine(x0, y0, z0, x1, y1, z1, r, g, b) {
        positions.push(x0, y0, z0);
        colors.push(r, g, b);
        positions.push(x1, y1, z1);
        colors.push(r, g, b);
    }

    const FREEBOARD = S * 1.0;
    const mastY     = -1.0 + S * 6.01;               // mast step 5m from bow
    const mastTopZ  = FREEBOARD + S * 16.705;       // masthead
    const fstayZ    = FREEBOARD + S * 15.14;        // forestay attachment
    const bowY      = -1.0;
    const spritEnd  = bowY - S * 1.8;               // bowsprit tip

    // Rigging colours
    const WIRE  = [0.35, 0.35, 0.35];   // shrouds / stays (dark grey)
    const SPREADER_COL = [0.08, 0.08, 0.10];  // spreaders (black)

    // ----- Forestay -----
    addLine(0, mastY, fstayZ, 0, bowY, FREEBOARD, ...WIRE);

    // (No backstay on this boat)

    // ----- Bobstay (bowsprit tip down to bow at waterline) -----
    addLine(0, spritEnd, FREEBOARD, 0, bowY, 0, ...WIRE);

    // ----- Spreader geometry -----
    const spSweepRad = 27 * Math.PI / 180;

    // First spreaders: same width as the beam (half-beam = 1.97m)
    const sp1Half    = S * 1.8;                        // first spreader half-span = half-beam
    const sp1SweepY  = Math.tan(spSweepRad) * sp1Half;  // aft offset at tip

    // Second spreaders: narrower
    const sp2Half    = S * 1.3;                       // second spreader half-span
    const sp2SweepY  = Math.tan(spSweepRad) * sp2Half;  // aft offset at tip

    // Heights along mast (fractions of P = 15.705m)
    const sp1Z = FREEBOARD + S * 15.705 * 0.32;   // first spreaders
    const sp2Z = FREEBOARD + S * 15.705 * 0.6;   // second spreaders

    // Spreader tips
    const sp1X  = sp1Half;
    const sp1Y  = mastY + sp1SweepY;
    const sp2X  = sp2Half;
    const sp2Y  = mastY + sp2SweepY;

    // Cap shroud top attachment (just below forestay)
    const capTopZ = fstayZ - S * 0.5;

    // Chainplate positions (deck edge, slightly aft of mast)
    const cpY = mastY + sp1SweepY;
    const cpX = 0.30;

    // Lower shroud chainplate (slightly further aft)
    const lcpY = cpY + S * 0.3;

    for (const side of [-1, 1]) {
        // Cap shrouds: top -> spreader 2 -> spreader 1 -> chainplate
        addLine(0, mastY, capTopZ,
                side * sp2X, sp2Y, sp2Z, ...WIRE);
        addLine(side * sp2X, sp2Y, sp2Z,
                side * sp1X, sp1Y, sp1Z, ...WIRE);
        addLine(side * sp1X, sp1Y, sp1Z,
                side * cpX, cpY, FREEBOARD, ...WIRE);

        // Lower shrouds: mast at sp1 height -> chainplate
        addLine(0, mastY, sp1Z,
                side * cpX, lcpY, FREEBOARD, ...WIRE);

        // Diagonals: first spreader tip -> second spreader root (at mast)
        addLine(side * sp1X, sp1Y, sp1Z,
                0, mastY, sp2Z, ...WIRE);

        // Spreader struts (mast to tip) — black
        addLine(0, mastY, sp1Z,
                side * sp1X, sp1Y, sp1Z, ...SPREADER_COL);
        addLine(0, mastY, sp2Z,
                side * sp2X, sp2Y, sp2Z, ...SPREADER_COL);
    }

    // =========================================================================
    // Create WebGL buffers
    // =========================================================================
    const posArr = new Float32Array(positions);
    const colArr = new Float32Array(colors);

    const positionBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, positionBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, posArr, gl.STATIC_DRAW);

    const colorBuffer = gl.createBuffer();
    gl.bindBuffer(gl.ARRAY_BUFFER, colorBuffer);
    gl.bufferData(gl.ARRAY_BUFFER, colArr, gl.STATIC_DRAW);

    return {
        position: positionBuffer,
        color: colorBuffer,
        numVertices: posArr.length / 3
    };
}

// pure 🍝
var instruction;

function initMaze() {
    var mice = [];
    var vips = [];
    var end;
    var nodes = {};
    var data = [];
    var xMin = 0;
    var xMax = 0;
    var yMin = 0;
    var yMax = 0;

    fetch("data/data.txt")
        .then((response) => {
              return response.text();
        })
        .then((queryString) => {
            const urlParams = new URLSearchParams(queryString),
                entries = urlParams.entries();

            for(const entry of entries) {
                data = entry[1].split(',');

                switch (entry[0]) {
                    case 'mouse':
                        mice.push({
                            x: parseInt(data[0]),
                            y: parseInt(data[1]),
                            facing: data[2],
                            name: data[3],
                            state: data[4],
                            vip: data[5]
                        });
                        break;

                    case 'vip':
                        vips.push({
                            x: parseInt(data[0]),
                            y: parseInt(data[1]),
                            name: data[2]
                        })
                        break;

                    case 'end':
                        end = {
                            x: parseInt(data[0]),
                            y: parseInt(data[1])
                        };
                        break;

                    case 'i':
                        instruction = {
                            x: parseInt(data[0]),
                            y: parseInt(data[1]),
                            facing: data[2],
                            i: data[3]
                        }
                        break;
                    
                    default:
                        var key = entry[0].split(',');
                        var x = parseInt(key[0]), 
                            y = parseInt(key[1]);

                        if (!nodes[x]) {
                            nodes[x] = [];
                        }

                        nodes[x][y] = {
                            n: data.indexOf('n') != -1,
                            e: data.indexOf('e') != -1, 
                            s: data.indexOf('s') != -1, 
                            w: data.indexOf('w') != -1,
                            ne: data.indexOf('ne') != -1,
                            en: data.indexOf('en') != -1, 
                            se: data.indexOf('se') != -1,
                            es: data.indexOf('es') != -1, 
                            sw: data.indexOf('sw') != -1,
                            ws: data.indexOf('ws') != -1, 
                            nw: data.indexOf('nw') != -1,
                            wn: data.indexOf('wn') != -1
                        };

                        if (x < xMin) xMin = x;
                        else if (x > xMax) xMax = x;
                        
                        if (y < yMin) yMin = y;
                        else if (y > yMax) yMax = y;
                }
            }

            generateMaze(xMin, xMax, yMin, yMax);
            displayMaze(mice, vips, end, instruction, nodes);
            showPath();
        });
}




function displayMaze(mice, vips, end, instruction, nodes) {
    var state = '';
    var keys = Object.keys(nodes);
    var x;

    for (var i = 0; i < keys.length; i++) {
        x = keys[i];
        for (var y = 0; y < nodes[x].length; y++) {
            if (nodes[x][y]) {
                addNode(x,y);
                addPaths(x, y, nodes[x][y]);
            }
        }
    }

    if (end) state += addEnd(end);
    if (vips.length) {
        state += `<b>VIPs</b><br>`;
        for (var v = 0; v < vips.length; v++) state += addVip(vips[v]);
        state += `<br>`
    }
    for (var m = 0; m < mice.length; m++) {
        state += addMouse(mice[m]);
        if (mice.length != m+1) state += `<br>`
    }
    if (instruction) state += `<br><b>Instruction</b><br>${instruction.i}<br>From (${instruction.x},${instruction.y}) facing ${instruction.facing.toUpperCase()}`;
    document.getElementById('state').innerHTML = state;
}




function addNode(x, y) {
    var cell = document.querySelector(`#cell-${x}-${y} > div`);

    cell.style.backgroundImage = `url('img/nodes/node.svg')`;
}

function addPaths(x, y, node) {
    var cell = document.querySelector(`#cell-${x}-${y} > div`);
    var keys = Object.keys(node);
    var d = '';
    var span = document.createElement('span');

    backgroundImage = cell.style.backgroundImage;

    for (var i = 0; i < keys.length; i++) {
        if (node[keys[i]]) {
            backgroundImage = `${backgroundImage}, url('img/paths/${keys[i]}.svg')`;
            d += `${keys[i].toUpperCase()} `;
        }
    }

    cell.style.backgroundImage = backgroundImage;
    cell = document.querySelector(`#cell-${x}-${y}`);
    cell.classList.add('tooltip');
    cell.setAttribute('onmouseover', `viewNode(${x},${y},true,'${d.toUpperCase().trim()}')`);
    cell.setAttribute('onmouseout', `viewNode(${x},${y},false,'${d.toUpperCase().trim()}')`);
    span.classList.add('tooltiptext');
    span.innerHTML = `(${x},${y})<br>${d.toUpperCase().trim()}`;
    cell.appendChild(span);
}

function addMouse(mouse) {
    var state = `<b>${mouse.name}</b><br>${mouse.state}<br>VIP: ${mouse.vip}<br>`;
    
    if (mouse.state != 'Idle' && mouse.state != 'Complete') {
        state += `Position: (${mouse.x},${mouse.y})<br>Facing: ${mouse.facing.toUpperCase()}<br>`;
        var cell = document.querySelector(`#cell-${mouse.x}-${mouse.y} > div`);
        cell.style.backgroundImage = `url('img/mice/mouse-${mouse.facing}.svg'), ${cell.style.backgroundImage}`;
        cell.style.zIndex = 999;
    }

    return state;
}

function addVip(vip) {
    var cell = document.querySelector(`#cell-${vip.x}-${vip.y} > div`);

    cell.style.backgroundImage = `url('img/vips/${vip.name}.png'), ${cell.style.backgroundImage}`;
    cell.style.zIndex = 999;

    return `${vip.name}: (${vip.x},${vip.y})<br>`;
}

function addEnd(end) {
    var cell = document.querySelector(`#cell-${end.x}-${end.y} > div`);

    cell.style.backgroundImage = `${cell.style.backgroundImage}, url('img/nodes/end.svg')`;

    return `<b>End </b><br>(${end.x},${end.y})<br><br>`;
}




function generateMaze(xMin, xMax, yMin, yMax) {    
    var div;
    var div2;
    var div3;

    for (var x = xMin; x <= xMax; x++) {
        div = document.createElement('div');
        for (var y = yMax; y >= yMin; y--) {
            div2 = document.createElement('div');
            div2.id = `cell-${x}-${y}`;
            div3 = document.createElement('div');
            div2.appendChild(div3);
            div.appendChild(div2);
        }
        document.getElementById('maze').appendChild(div);
    }
}




function viewNode(x,y,on) {
    var color = on ? `#10101011`:`transparent`;

    document.querySelector(`#cell-${x}-${y} > div`).style.backgroundColor = color;
}



instructionDataSets = {
    'NL': {
        facing: 'w',
        x: 0,
        y: 0
    }, 'SR': {
        facing: 'w',
        x: 0,
        y: 0
    }, 'EB': {
        facing: 'w',
        x: 0,
        y: 0
    }, 'EL': {
        facing: 'n',
        x: 0,
        y: 0
    }, 'WR': {
        facing: 'n',
        x: 0,
        y: 0
    }, 'SB': {
        facing: 'n',
        x: 0,
        y: 0
    }, 'SL': {
        facing: 'e',
        x: 0,
        y: 0
    }, 'NR': {
        facing: 'e',
        x: 0,
        y: 0
    }, 'WB': {
        facing: 'e',
        x: 0,
        y: 0
    }, 'WL': {
        facing: 's',
        x: 0,
        y: 0
    }, 'ER': {
        facing: 's',
        x: 0,
        y: 0
    }, 'NB': {
        facing: 's',
        x: 0,
        y: 0
    }, 'NF': {
        facing: 'n',
        x: 0,
        y: 1
    }, 'EF': {
        facing: 'e',
        x: 1,
        y: 0
    }, 'SF': {
        facing: 's',
        x: 0,
        y: -1
    }, 'WF': {
        facing: 'w',
        x: -1,
        y: 0
    }
}
async function showPath() {
    var x = instruction.x;
    var y = instruction.y;
    var facing = instruction.facing;
    var inst = Array.from(instruction.i);
    var cell, cellBackground;
    var div;
    var mouseClass

    document.querySelector('a').classList.add('hide');

    for (var i = 0; i < inst.length; i++) {
        if (document.querySelector('.mouse')) document.querySelector('.mouse').remove();

        cell = document.querySelector(`#cell-${x}-${y}`);
        cellBackground = document.querySelector(`#cell-${x}-${y} > div`).style.backgroundImage;

        div = document.createElement('div');
        div.style.backgroundImage = `url('img/mice/ghost-mouse-${facing}.svg')`;
        div.style.zIndex = 1000;
        div.classList.add('mouse');

        mouseClass = facing.toUpperCase() + inst[i];

        if (mouseClass.indexOf('C') == -1) {
            instructionData = instructionDataSets[mouseClass];
            x += instructionData.x;
            y += instructionData.y;
            facing = instructionData.facing;
        } else {
            console.log(x,y,mouseClass, cellBackground);
            switch (mouseClass) {
                case 'NC':
                    if (cellBackground.indexOf('ne') != -1) {
                        mouseClass += 'R';
                        x += 1;
                        y += 1;
                        facing = 'e';
                    } else {
                        mouseClass += 'L';
                        x += -1;
                        y += 1;
                        facing = 'w';
                    }
                    break;

                case 'EC':
                    if (cellBackground.indexOf('en') != -1) {
                        mouseClass += 'L';
                        x += 1;
                        y += 1;
                        facing = 'n';
                    } else {
                        mouseClass += 'R';
                        x += 1;
                        y += -1;
                        facing = 's';
                    }
                    break;

                case 'SC':
                    if (cellBackground.indexOf('sw') != -1) {
                        mouseClass += 'R';
                        x += -1;
                        y += -1;
                        facing = 'w';
                    } else {
                        mouseClass += 'L';
                        x += 1;
                        y += -1;
                        facing = 'e';
                    }
                    break;

                case 'WC':
                    if (cellBackground.indexOf('ws') != -1) {
                        mouseClass += 'L';
                        x += -1;
                        y += -1;
                        facing = 's';
                    } else {
                        mouseClass += 'R';
                        x += -1;
                        y += 1;
                        facing = 'n';
                    }
                    break;
            }
        }

        div.classList.add(mouseClass);
        cell.appendChild(div);

        await new Promise(resolve => setTimeout(resolve, 1000));
    }

    if (document.querySelector('.mouse')) document.querySelector('.mouse').remove();
    document.querySelector('a').classList.remove('hide');
}

initMaze();
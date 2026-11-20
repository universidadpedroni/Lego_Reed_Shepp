// Prueba de logica del dashboard con jsdom (npm i jsdom). Uso: node SMC/test_dashboard.js
const {JSDOM} = require('jsdom');
const fs = require('fs');
let html = fs.readFileSync(process.env.HOME + '/mnt/Lego_Reed_Shepp/dashboard.html', 'utf8');
html = html.replace(/<script src="[^"]*chart[^"]*"><\/script>/, '');
class FakeChart { constructor(){ this.data={datasets:[{data:[],label:'a'},{data:[],label:'b'}]}; this.options={scales:{x:{},y:{}},plugins:{legend:{labels:{}}}}; } update(){} }
const dom = new JSDOM(html, {runScripts:'dangerously', pretendToBeVisual:true, url:'http://localhost/', beforeParse(w){ w.Chart = FakeChart; w.localStorage.clear(); }});
const w = dom.window;
w.addEventListener('error', e => { console.log('ERROR JS:', e.message); process.exitCode = 1; });
setTimeout(async () => {
  const $ = id => w.document.getElementById(id);
  const sent = [];
  w.sendCommand = async c => { sent.push(c); };
  w.sleep = async () => {}; w.waitQuiet = async () => {};
  const fails = [];
  const chk = (c, m) => { if (!c) fails.push(m); };
  chk(!$('ctrlSel') && !$('smcBox') && !$('algSel'), 'quedaron controles de SMC/Dubins');
  // iniciar ensayo (hub "listo")
  w.eval('hubReady=true;connected=true;');
  await w.onStart();
  console.log('enviados:', sent.join(' | '));
  chk(sent.includes('PI 50,0.01,50,0.01'), 'PI');
  chk(!sent.some(x => /^(ALG|CTRL|SMC)/.test(x)), 'no debe mandar ALG/CTRL/SMC');
  chk(sent[sent.length-1] === 'START', 'START ultimo');
  // telemetria 17 campos + LOOP + fin
  for (let k = 0; k < 20; k++) {
    const t = (k*0.1).toFixed(2);
    w.handleLine('T ' + [t,5,4.5,0.5,40,10,9,1,-60+k,3,4,20,5,-55,-5,(0.3*Math.sin(k)).toFixed(2),0.01].join(','));
  }
  w.handleLine('LOOP 1.37');
  w.handleLine('END ok');
  console.log('lat:', $('mLat').textContent, '| act:', $('mAct').textContent, '| loop:', $('mLoop').textContent);
  console.log('hist:', $('hist').textContent);
  chk($('mLat').textContent.includes('cm'), 'mLat');
  chk($('mLoop').textContent === '1.37', 'mLoop');
  chk($('hist').textContent.includes('PI'), 'hist');
  // telemetria vieja de 15 campos no rompe
  w.handleLine('T 0.1,5,4.5,0.5,40,10,9,1,-60,3,4,20,5,-55,-5');
  console.log(fails.length ? 'FALLAS: ' + fails.join('; ') : 'DASHBOARD OK');
  process.exit(fails.length ? 1 : 0);
}, 300);

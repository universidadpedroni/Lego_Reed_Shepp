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
  // estado inicial
  chk($('smcBox').style.display === 'none', 'smcBox visible por defecto');
  chk($('tagDirP').textContent === 'P', 'tag P');
  // elegir SMC
  $('ctrlSel').value = 'SMC'; $('ctrlSel').dispatchEvent(new w.Event('change'));
  chk($('smcBox').style.display === 'block', 'smcBox no aparece');
  chk($('tagDirP').textContent === 'Eq' && $('tagDirI').textContent === 'Sw', 'tags Eq/Sw');
  // iniciar ensayo (hub "listo")
  w.eval('hubReady=true;connected=true;');
  await w.onStart();
  console.log('enviados:', sent.join(' | '));
  chk(sent.indexOf('CTRL SMC') > sent.indexOf('PI 50,0.01,50,0.01') && sent.indexOf('CTRL SMC') >= 0, 'orden CTRL');
  chk(sent.includes('SMC 0.035,0.008,0.05,0,0.2,20'), 'SMC params');
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
  chk($('hist').textContent.includes('SMC'), 'hist');
  // PI: no manda SMC
  sent.length = 0; $('ctrlSel').value = 'PI'; $('ctrlSel').dispatchEvent(new w.Event('change'));
  w.eval('running=false;'); await w.onStart();
  chk(sent.includes('CTRL PI') && !sent.some(x => x.startsWith('SMC ')), 'PI no debe mandar SMC');
  // telemetria vieja de 15 campos no rompe
  w.handleLine('T 0.1,5,4.5,0.5,40,10,9,1,-60,3,4,20,5,-55,-5');
  console.log(fails.length ? 'FALLAS: ' + fails.join('; ') : 'DASHBOARD OK');
  process.exit(fails.length ? 1 : 0);
}, 300);

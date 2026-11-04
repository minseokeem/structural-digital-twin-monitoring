const dgram = require("dgram");

const HOST = process.env.SENSOR_SERVER_IP || "127.0.0.1";
const PORT = Number(process.env.SENSOR_UDP_PORT || 4210);
const socket = dgram.createSocket("udp4");

const anchors = ["ANCHOR-01","ANCHOR-02","ANCHOR-03","ANCHOR-04"];
const tags = ["TAG-01"];

const baseRanges = {
  "TAG-01":[3.8,5.1,6.0,6.7],
  "TAG-02":[4.5,4.2,5.2,5.9],
  "TAG-03":[5.7,5.1,4.3,4.8],
  "TAG-04":[6.5,5.9,5.0,3.9]
};

let seq = 0;
const t0 = Date.now();

function send(obj) {
  const b = Buffer.from(JSON.stringify(obj));
  socket.send(b, PORT, HOST);
}

function tick() {
  const t = (Date.now()-t0)/1000;
  seq++;

  tags.forEach((id,i)=>{
    const dz = 0.004*Math.sin(t*.55+i*.8); // ~4 mm equivalent range variation
    const ranges = anchors.map((a,j)=>({
      anchor_id:a,
      ok:true,
      m:baseRanges[id][j] + dz*(0.75+0.08*j),
      success_rate:100,
      recent_successes:32,
      recent_samples:32,
      attempt_success_rate:100,
      attempts:seq,
      successes:seq
    }));

    send({
      type:"tag_data",
      node_id:id,
      role:"tag",
      seq,
      tilt_deg:1.8*Math.sin(t*.45+i*.7),
      vibration:Math.sin(t*2.2+i)>.97,
      dynamic_g:Math.abs(.08*Math.sin(t*2.2+i)),
      uwb_active:true,
      ranges,
      wifi_rssi:-47-i*2,
      cmd_port:4211,
      fw:"soc-web-dw3000-tag-v5-mock"
    });
  });

  if (seq % 10 === 0) {
    console.log(`[mock] sent ${tags.length} Tag with ${anchors.length} relayed UWB Anchors -> ${HOST}:${PORT}`);
  }
}

console.log("SOC DW3000 mock sensor sender");
console.log(`target: ${HOST}:${PORT}`);
console.log("Ctrl+C to stop.");
tick();
const timer=setInterval(tick,100);

process.on("SIGINT",()=>{
  clearInterval(timer);
  socket.close(()=>process.exit(0));
});

import fetch from 'node-fetch';
const { Writable } = require("stream");


// Credit: https://gist.github.com/ornicar/a097406810939cf7be1df8ea30e94f3e
const readStream = processLine => response => {
  const stream = response.body;
  const matcher = /\r?\n/;
  const decoder = new TextDecoder();
  const writable = new Writable();
  let buf = '';

  /*
  const loop = () =>
    // .read() will return a promise with the raw data (i.e. in bytes) of the stream data
    stream.read().then(({ done, value }) => {
      if (done) {
        if (buf.length > 0) processLine(JSON.parse(buf));
      } else {
        const chunk = decoder.decode(value, {
          stream: true
        });
        buf += chunk;

        const parts = buf.split(matcher);
        buf = parts.pop();
        for (const i of parts.filter(p => p)) processLine(JSON.parse(i));
        return loop();
      }
    });

  return loop();
  */
}

const PAT = "lip_T2E2rY0sd498e8DW559z";
let game_id = "";
async function event_request() {
    const response = await fetch("https://lichess.org/api/stream/event", {
        method: "GET",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then((res) => res.text()
    .then((text) => {console.debug(text);}))
    .catch((e) => {console.debug(e);});
}

async function challenge_request() {
    const response = await fetch("https://lichess.org/api/challenge/ai", {
        method: "POST",
        headers: {
            "Content-Type": "application/x-www-form-urlencoded",
            "Authorization": `Bearer ${PAT}`
        },
        body: "level=1&days=1"
    })
    .then((res) => res.json()
    .then((json) => {
        game_id = json.id;
        console.debug(json);
    }));

    await move_request();
}

async function move_request() {
    const response = await fetch(`https://lichess.org/api/board/game/stream/${game_id}`, {
        method: "GET",
        headers: {
            "Authorization": `Bearer ${PAT}`
        }
    })
    .then((res) => {
        console.debug(`${res.status}: ${res.statusText}`);
        console.debug(`${res.body}`);
        readStream((string) => { console.debug(string); })(res);
    }).catch((e) => {
        console.debug("got error");
        console.debug(e);
    });
        
    console.debug("returning");
}

event_request();
challenge_request();

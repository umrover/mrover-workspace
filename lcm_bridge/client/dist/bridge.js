export default class LCMBridge {
    constructor (url, updateWebsocketState, updateConnectedState, 
                 lcmMessage, subscriptions) {
        this.online = false
        this.subscriptions = subscriptions
        this.updateWebsocketState = updateWebsocketState
        this.updateConnectedState = updateConnectedState
        this.lcmMessage = lcmMessage
        this._createWebsocket(url)
        this.callbacks = {}
    }

    _createWebsocket (url) {
        this.ws = new WebSocket(url)

        this.ws.onclose = (event) => {
            this.online = false
            this.updateWebsocketState(this.online)
            this.updateConnectedState([false, false])

            setTimeout(() => {
                this._createWebsocket(url)
            }, 2*1000)
        }

        this.ws.onopen = (event) => {
            this.online = true
            this.updateWebsocketState(this.online)

            // Make subscriptions
            /*for (const sub in this.subscriptions) {
                this._subscribe(sub.topic, sub.type)
            }*/
            this.subscriptions.forEach(this._subscribe, this)
        }

        this.ws.onmessage = (event) => {
            let event_data = JSON.parse(event.data)
            if (event_data['type'] === 'connection_state') {
                this.updateConnectedState(event_data['state'])
            }
            if (event_data['type'] === 'lcm_message') {
                this.lcmMessage({
                    'topic': event_data['topic'],
                    'message': event_data['message']
                })

                if(this.callbacks[event_data['topic']] !== undefined) {
                    this.callbacks[event_data['topic']].forEach(function(fn){
                        fn(event_data['message'])
                    })
                }
            }
            if (event_data['type'] === 'error_message'){
                console.error(event_data['message']);
            }
        }
    }

    _subscribe ({topic, type}) {
        this.ws.send(JSON.stringify({
            'type': 'lcm_subscribe',
            'topic': topic,
            'lcm_type': type
        }))
    }

    publish (topic, message) {
        if (this.online) {
            this.ws.send(JSON.stringify({
                'type': 'lcm_publish',
                'topic': topic,
                'message': message
            }))
        } else {
            console.error("LCM Bridge not connected")
        }
    }

    subscribe(channel, callbackFn) {
        if(this.callbacks[channel] === undefined){
            this.callbacks[channel] = [callbackFn]
        }else{
            this.callbacks[channel].push(callbackFn)
        }
    }

    setHomePage(){
        if(this.online){
            this.ws.send(JSON.stringify({
                'type':'home_page_set'
            }))
        }
    }
}

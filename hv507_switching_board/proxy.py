from path_helpers import path
try:
    from .node import (Proxy as _Proxy, I2cProxy as _I2cProxy,
                       SerialProxy as _SerialProxy)

    class ProxyMixin(object):
        '''
        Mixin class to add convenience wrappers around methods of the generated
        `node.Proxy` class.

        For example, expose config and state getters/setters as attributes.
        '''
        host_package_name = str(path(__file__).parent.name.replace('_', '-'))

        @property
        def config(self):
            from .config import Config

            return Config.FromString(self.serialize_config().tostring())

        @config.setter
        def config(self, value):
            return self.update_config(value)

        @property
        def state(self):
            from .config import State

            return State.FromString(self.serialize_state().tostring())

        @state.setter
        def state(self, value):
            return self.update_state(value)

        def update_config(self, **kwargs):
            '''
            Update fields in the config object based on keyword arguments.

            By default, these values will be saved to EEPROM. To prevent this
            (e.g., to verify system behavior before committing the changes),
            you can pass the special keyword argument 'save=False'. In this case,
            you will need to call the method save_config() to make your changes
            persistent.
            '''

            from .config import Config

            save = True
            if 'save' in kwargs.keys() and not kwargs.pop('save'):
                save = False

            config = Config(**kwargs)
            return_code = super(ProxyMixin, self).update_config(config)

            if save:
                super(ProxyMixin, self).save_config()

            return return_code

        def update_state(self, **kwargs):
            from .config import State

            state = State(**kwargs)
            return super(ProxyMixin, self).update_state(state)
        
        def _state_of_channels(self):
            '''
            Prepend underscore to the auto-generated state_of_channels accessor
            '''
            return super(ProxyMixin, self).state_of_channels()

        def _set_state_of_channels(self, states):
            '''
            Prepend underscore to the auto-generated state_of_channels setter
            '''
            return super(ProxyMixin, self).set_state_of_channels(states)

        @property
        def state_of_channels(self):
            '''
            Retrieve the state bytes from the device and unpacks them into an
            array with one entry per channel.  Return unpacked array.

            Notes
            -----

            State of each channel is binary, 0 or 1.  On device, states are
            stored in bytes, where each byte corresponds to the state of eight
            channels.
            '''
            import numpy as np

            return np.unpackbits(super(ProxyMixin, self).state_of_channels())

        @state_of_channels.setter
        def state_of_channels(self, states):
            self.set_state_of_channels(states)

        def set_state_of_channels(self, states):
            '''
            Pack array containing one entry per channel to bytes (8 channels
            per byte).  Set state of channels on device using state bytes.

            See also: `state_of_channels` (get)
            '''
            import numpy as np
            
            ok =  (super(ProxyMixin, self)
                    .set_state_of_channels(np.packbits(states.astype(int))))
            if not ok:
                raise ValueError('Error setting state of channels.  Check '
                                 'number of states matches channel count.')
                
        @property
        def baud_rate(self):
            return self.config.baud_rate

        @baud_rate.setter
        def baud_rate(self, baud_rate):
            return self.update_config(baud_rate=baud_rate)

        @property
        def serial_number(self):
            return self.config.serial_number

        @serial_number.setter
        def serial_number(self, serial_number):
            return self.update_config(serial_number=serial_number)
        
        @property
        def port(self):
            return self._stream.serial_device.port

        @port.setter
        def port(self, port):
            return self.update_config(port=port)
        
        def _channel_count(self):
            return super(ProxyMixin, self).channel_count()
        
        @property
        def channel_count(self):
            return self._channel_count()
        
        @property
        def hardware_version(self):
            '''
            Placeholder for a remote hardware_version accessor
            '''
            return '0.1'


    class Proxy(ProxyMixin, _Proxy):
        pass

    class I2cProxy(ProxyMixin, _I2cProxy):
        pass

    class SerialProxy(ProxyMixin, _SerialProxy):
        pass

except (ImportError, TypeError):
    Proxy = None
    I2cProxy = None
    SerialProxy = None

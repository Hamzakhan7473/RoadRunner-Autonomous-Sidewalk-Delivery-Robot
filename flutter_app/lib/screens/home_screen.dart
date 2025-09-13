import 'package:flutter/material.dart';
import 'package:provider/provider.dart';
import 'package:google_maps_flutter/google_maps_flutter.dart';
import 'package:socket_io_client/socket_io_client.dart' as IO;

import '../localization/app_localizations.dart';
import '../providers/robot_provider.dart';
import '../providers/language_provider.dart';
import '../widgets/robot_status_card.dart';
import '../widgets/mission_list.dart';
import '../widgets/teleop_controls.dart';
import '../widgets/navigation_map.dart';
import '../utils/app_constants.dart';

class HomeScreen extends StatefulWidget {
  const HomeScreen({super.key});

  @override
  State<HomeScreen> createState() => _HomeScreenState();
}

class _HomeScreenState extends State<HomeScreen> with TickerProviderStateMixin {
  late TabController _tabController;
  IO.Socket? _socket;
  bool _isConnected = false;

  @override
  void initState() {
    super.initState();
    _tabController = TabController(length: 4, vsync: this);
    _initializeSocketConnection();
  }

  void _initializeSocketConnection() {
    try {
      _socket = IO.io('http://localhost:5000', <String, dynamic>{
        'transports': ['websocket'],
        'autoConnect': false,
      });

      _socket!.onConnect((_) {
        setState(() {
          _isConnected = true;
        });
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text(AppLocalizations.of(context).robotConnected),
            backgroundColor: Colors.green,
          ),
        );
      });

      _socket!.onDisconnect((_) {
        setState(() {
          _isConnected = false;
        });
        ScaffoldMessenger.of(context).showSnackBar(
          SnackBar(
            content: Text(AppLocalizations.of(context).robotDisconnected),
            backgroundColor: Colors.red,
          ),
        );
      });

      _socket!.on('robot_status', (data) {
        // Handle robot status updates
        if (mounted) {
          Provider.of<RobotProvider>(context, listen: false).updateRobotStatus(data);
        }
      });

      _socket!.on('navigation_context', (data) {
        // Handle navigation context updates
        if (mounted) {
          Provider.of<RobotProvider>(context, listen: false).updateNavigationContext(data);
        }
      });

      _socket!.connect();
    } catch (e) {
      ScaffoldMessenger.of(context).showSnackBar(
        SnackBar(
          content: Text('${AppLocalizations.of(context).connectionError}: $e'),
          backgroundColor: Colors.red,
        ),
      );
    }
  }

  @override
  void dispose() {
    _tabController.dispose();
    _socket?.disconnect();
    super.dispose();
  }

  @override
  Widget build(BuildContext context) {
    final localizations = AppLocalizations.of(context);
    
    return Scaffold(
      appBar: AppBar(
        title: Text(localizations.appTitle),
        backgroundColor: AppConstants.primaryColor,
        foregroundColor: Colors.white,
        actions: [
          // Connection status indicator
          Container(
            margin: const EdgeInsets.only(right: 16),
            child: Row(
              children: [
                Icon(
                  _isConnected ? Icons.wifi : Icons.wifi_off,
                  color: _isConnected ? Colors.green : Colors.red,
                ),
                const SizedBox(width: 8),
                Text(
                  _isConnected ? 'Connected' : 'Disconnected',
                  style: TextStyle(
                    color: _isConnected ? Colors.green : Colors.red,
                    fontSize: 12,
                  ),
                ),
              ],
            ),
          ),
          // Language selector
          PopupMenuButton<String>(
            icon: const Icon(Icons.language),
            onSelected: (String languageCode) {
              Provider.of<LanguageProvider>(context, listen: false)
                  .setLanguage(languageCode);
            },
            itemBuilder: (BuildContext context) => [
              PopupMenuItem<String>(
                value: 'en',
                child: Text(localizations.english),
              ),
              PopupMenuItem<String>(
                value: 'hi',
                child: Text(localizations.hindi),
              ),
              PopupMenuItem<String>(
                value: 'ta',
                child: Text(localizations.tamil),
              ),
            ],
          ),
          // Settings
          IconButton(
            icon: const Icon(Icons.settings),
            onPressed: () {
              _showSettingsDialog(context);
            },
          ),
        ],
        bottom: TabBar(
          controller: _tabController,
          tabs: [
            Tab(
              icon: const Icon(Icons.home),
              text: localizations.home,
            ),
            Tab(
              icon: const Icon(Icons.local_shipping),
              text: localizations.delivery,
            ),
            Tab(
              icon: const Icon(Icons.gamepad),
              text: localizations.teleop,
            ),
            Tab(
              icon: const Icon(Icons.map),
              text: localizations.route,
            ),
          ],
        ),
      ),
      body: TabBarView(
        controller: _tabController,
        children: [
          // Home Tab - Robot Status and Overview
          _buildHomeTab(),
          // Delivery Tab - Mission Management
          _buildDeliveryTab(),
          // Teleop Tab - Manual Control
          _buildTeleopTab(),
          // Map Tab - Navigation and Route Planning
          _buildMapTab(),
        ],
      ),
      floatingActionButton: FloatingActionButton(
        onPressed: () {
          _showNewDeliveryDialog(context);
        },
        backgroundColor: AppConstants.primaryColor,
        child: const Icon(Icons.add, color: Colors.white),
      ),
    );
  }

  Widget _buildHomeTab() {
    return SingleChildScrollView(
      padding: const EdgeInsets.all(16),
      child: Column(
        crossAxisAlignment: CrossAxisAlignment.start,
        children: [
          // Robot Status Card
          const RobotStatusCard(),
          
          const SizedBox(height: 16),
          
          // Quick Actions
          Text(
            AppLocalizations.of(context).robotStatus,
            style: const TextStyle(
              fontSize: 20,
              fontWeight: FontWeight.bold,
            ),
          ),
          
          const SizedBox(height: 16),
          
          Row(
            children: [
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: () {
                    _socket?.emit('start_mission');
                  },
                  icon: const Icon(Icons.play_arrow),
                  label: Text(AppLocalizations.of(context).startMission),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.green,
                    foregroundColor: Colors.white,
                  ),
                ),
              ),
              const SizedBox(width: 16),
              Expanded(
                child: ElevatedButton.icon(
                  onPressed: () {
                    _socket?.emit('emergency_stop');
                  },
                  icon: const Icon(Icons.stop),
                  label: Text(AppLocalizations.of(context).emergencyStop),
                  style: ElevatedButton.styleFrom(
                    backgroundColor: Colors.red,
                    foregroundColor: Colors.white,
                  ),
                ),
              ),
            ],
          ),
          
          const SizedBox(height: 16),
          
          // Recent Missions
          Text(
            'Recent Missions',
            style: const TextStyle(
              fontSize: 18,
              fontWeight: FontWeight.bold,
            ),
          ),
          
          const SizedBox(height: 8),
          
          const MissionList(),
        ],
      ),
    );
  }

  Widget _buildDeliveryTab() {
    return const MissionList();
  }

  Widget _buildTeleopTab() {
    return const TeleopControls();
  }

  Widget _buildMapTab() {
    return const NavigationMap();
  }

  void _showNewDeliveryDialog(BuildContext context) {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Text(AppLocalizations.of(context).newDelivery),
          content: const NewDeliveryForm(),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: Text(AppLocalizations.of(context).cancel),
            ),
            ElevatedButton(
              onPressed: () {
                // Handle new delivery creation
                Navigator.of(context).pop();
              },
              child: Text(AppLocalizations.of(context).confirm),
            ),
          ],
        );
      },
    );
  }

  void _showSettingsDialog(BuildContext context) {
    showDialog(
      context: context,
      builder: (BuildContext context) {
        return AlertDialog(
          title: Text(AppLocalizations.of(context).settings),
          content: Column(
            mainAxisSize: MainAxisSize.min,
            children: [
              ListTile(
                leading: const Icon(Icons.language),
                title: Text(AppLocalizations.of(context).language),
                trailing: const Icon(Icons.arrow_forward_ios),
                onTap: () {
                  Navigator.of(context).pop();
                  // Show language selection
                },
              ),
              ListTile(
                leading: const Icon(Icons.notifications),
                title: const Text('Notifications'),
                trailing: Switch(
                  value: true,
                  onChanged: (value) {
                    // Handle notification toggle
                  },
                ),
              ),
              ListTile(
                leading: const Icon(Icons.dark_mode),
                title: const Text('Dark Mode'),
                trailing: Switch(
                  value: false,
                  onChanged: (value) {
                    // Handle dark mode toggle
                  },
                ),
              ),
            ],
          ),
          actions: [
            TextButton(
              onPressed: () => Navigator.of(context).pop(),
              child: Text(AppLocalizations.of(context).cancel),
            ),
          ],
        );
      },
    );
  }
}

class NewDeliveryForm extends StatefulWidget {
  const NewDeliveryForm({super.key});

  @override
  State<NewDeliveryForm> createState() => _NewDeliveryFormState();
}

class _NewDeliveryFormState extends State<NewDeliveryForm> {
  final _formKey = GlobalKey<FormState>();
  final _customerNameController = TextEditingController();
  final _customerPhoneController = TextEditingController();
  final _deliveryAddressController = TextEditingController();
  final _specialInstructionsController = TextEditingController();
  String _selectedPriority = 'medium';

  @override
  Widget build(BuildContext context) {
    final localizations = AppLocalizations.of(context);
    
    return Form(
      key: _formKey,
      child: SingleChildScrollView(
        child: Column(
          mainAxisSize: MainAxisSize.min,
          children: [
            TextFormField(
              controller: _customerNameController,
              decoration: InputDecoration(
                labelText: localizations.customerName,
                border: const OutlineInputBorder(),
              ),
              validator: (value) {
                if (value == null || value.isEmpty) {
                  return 'Please enter customer name';
                }
                return null;
              },
            ),
            
            const SizedBox(height: 16),
            
            TextFormField(
              controller: _customerPhoneController,
              decoration: InputDecoration(
                labelText: localizations.customerPhone,
                border: const OutlineInputBorder(),
              ),
              keyboardType: TextInputType.phone,
              validator: (value) {
                if (value == null || value.isEmpty) {
                  return 'Please enter phone number';
                }
                return null;
              },
            ),
            
            const SizedBox(height: 16),
            
            TextFormField(
              controller: _deliveryAddressController,
              decoration: InputDecoration(
                labelText: localizations.deliveryAddress,
                border: const OutlineInputBorder(),
              ),
              maxLines: 3,
              validator: (value) {
                if (value == null || value.isEmpty) {
                  return 'Please enter delivery address';
                }
                return null;
              },
            ),
            
            const SizedBox(height: 16),
            
            DropdownButtonFormField<String>(
              value: _selectedPriority,
              decoration: InputDecoration(
                labelText: localizations.priority,
                border: const OutlineInputBorder(),
              ),
              items: [
                DropdownMenuItem(
                  value: 'high',
                  child: Text(localizations.high),
                ),
                DropdownMenuItem(
                  value: 'medium',
                  child: Text(localizations.medium),
                ),
                DropdownMenuItem(
                  value: 'low',
                  child: Text(localizations.low),
                ),
              ],
              onChanged: (value) {
                setState(() {
                  _selectedPriority = value!;
                });
              },
            ),
            
            const SizedBox(height: 16),
            
            TextFormField(
              controller: _specialInstructionsController,
              decoration: InputDecoration(
                labelText: localizations.specialInstructions,
                border: const OutlineInputBorder(),
              ),
              maxLines: 2,
            ),
          ],
        ),
      ),
    );
  }
}
